/*
 * arch/gpt/kernel/topology.c
 *
 * Copyright (C) 2017, General Processor Techologies Inc. & HXGPT Tech Ltd.
 *
 * Based on the arm64/arm32 version written by Vincent Guittot in turn based on
 * arch/sh/kernel/topology.c
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */

#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/init.h>
#include <linux/percpu.h>
#include <linux/node.h>
#include <linux/nodemask.h>
#include <linux/of.h>
#include <linux/sched.h>

#include <asm/topology.h>

#define BITS_MASK_BY_WIDTH(width) ((1UL << (width)) - 1)

/*
 * cpu topology table
 */
struct cpu_topology cpu_topology[NR_CPUS];
EXPORT_SYMBOL_GPL(cpu_topology);

static int __init get_cpu_for_node(struct device_node *node)
{
	struct device_node *cpu_node;
	int cpu;

	cpu_node = of_parse_phandle(node, "cpu", 0);
	if (!cpu_node) {
		return -1;
	}

	for_each_possible_cpu(cpu) {
		if (of_get_cpu_node(cpu, NULL) == cpu_node) {
			of_node_put(cpu_node);
			return cpu;
		}
	}

	pr_crit("Unable to find CPU node for %s\n", cpu_node->full_name);

	of_node_put(cpu_node);
	return -1;
}

static int __init parse_core(struct device_node *core, int group_id, int core_id)
{
	char name[10];
	bool leaf = true;
	int i = 0;
	int cpu;
	struct device_node *t;

	do {
		snprintf(name, sizeof(name), "thread%d", i);
		t = of_get_child_by_name(core, name);
		if (t) {
			leaf = false;
			cpu = get_cpu_for_node(t);
			if (cpu >= 0) {
				cpu_topology[cpu].group_id = group_id;
				cpu_topology[cpu].core_id = core_id;
				cpu_topology[cpu].thread_id = i;
			} else {
				pr_err("%s: Can't get CPU for thread\n",
				       t->full_name);
				of_node_put(t);
				return -EINVAL;
			}
			of_node_put(t);
		}
		i++;
	} while (t);

	cpu = get_cpu_for_node(core);
	if (cpu >= 0) {
		if (!leaf) {
			pr_err("%s: Core has both threads and CPU\n",
			       core->full_name);
			return -EINVAL;
		}

		cpu_topology[cpu].group_id = group_id;
		cpu_topology[cpu].core_id = core_id;
	} else if (leaf) {
		pr_err("%s: Can't get CPU for leaf core\n", core->full_name);
		return -EINVAL;
	}

	return 0;
}

static int __init parse_group(struct device_node *group, int depth)
{
	char name[10];
	bool leaf = true;
	bool has_cores = false;
	struct device_node *c;
	static int group_id __initdata;
	int core_id = 0;
	int i, ret;

	/*
	 * First check for child groups; we currently ignore any
	 * information about the nesting of groups and present the
	 * scheduler with a flat list of them.
	 */
	i = 0;
	do {
		snprintf(name, sizeof(name), "group%d", i);
		c = of_get_child_by_name(group, name);
		if (c) {
			leaf = false;
			ret = parse_group(c, depth + 1);
			of_node_put(c);
			if (ret != 0) {
				return ret;
			}
		}
		i++;
	} while (c);

	/* Now check for cores */
	i = 0;
	do {
		snprintf(name, sizeof(name), "core%d", i);
		c = of_get_child_by_name(group, name);
		if (c) {
			has_cores = true;

			if (depth == 0) {
				pr_err("%s: cpu-map children should be groups\n",
				       c->full_name);
				of_node_put(c);
				return -EINVAL;
			}

			if (leaf) {
				ret = parse_core(c, group_id, core_id++);
			} else {
				pr_err("%s: Non-leaf group with core %s\n",
				       group->full_name, name);
				ret = -EINVAL;
			}

			of_node_put(c);
			if (ret != 0) {
				return ret;
			}
		}
		i++;
	} while (c);

	if (leaf && !has_cores) {
		pr_warn("%s: empty group\n", group->full_name);
	}

	if (leaf) {
		group_id++;
	}

	return 0;
}

static int __init parse_dt_topology(void)
{
	struct device_node *cn, *map;
	int ret = 0;
	int cpu;

	cn = of_find_node_by_path("/cpus");
	if (!cn) {
		pr_err("No CPU information found in DT\n");
		return 0;
	}

	/*
	 * When topology is provided cpu-map is essentially a root
	 * group with restricted subnodes.
	 */
	map = of_get_child_by_name(cn, "cpu-map");
	if (!map) {
		goto out;
	}

	ret = parse_group(map, 0);
	if (ret != 0) {
		goto out_map;
	}

	/*
	 * Check that all cores are in the topology; the SMP code will
	 * only mark cores described in the DT as possible.
	 */
	for_each_possible_cpu(cpu) {
		if (cpu_topology[cpu].group_id == -1) {
			ret = -EINVAL;
		}
	}

out_map:
	of_node_put(map);
out:
	of_node_put(cn);
	return ret;
}

const struct cpumask *cpu_coregroup_mask(int cpu)
{
	return &cpu_topology[cpu].core_map;
}

static void update_siblings_masks(unsigned int cpuid)
{
	struct cpu_topology *cpu_topo, *cpuid_topo = &cpu_topology[cpuid];
	int cpu;

	/* update core and thread sibling masks */
	for_each_possible_cpu(cpu) {
		cpu_topo = &cpu_topology[cpu];

		if (cpuid_topo->group_id != cpu_topo->group_id) {
			continue;
		}

		cpumask_set_cpu(cpuid, &cpu_topo->core_map);
		if (cpu != cpuid) {
			cpumask_set_cpu(cpu, &cpuid_topo->core_map);
		}

		if (cpuid_topo->core_id != cpu_topo->core_id) {
			continue;
		}

		cpumask_set_cpu(cpuid, &cpu_topo->thread_map);
		if (cpu != cpuid) {
			cpumask_set_cpu(cpu, &cpuid_topo->thread_map);
		}
	}
}

void store_cpu_topology(unsigned int cpuid)
{
	struct cpu_topology *cpuid_topo = &cpu_topology[cpuid];
	u64 thid;

	if (cpuid_topo->group_id != -1) {
		goto topology_populated;
	}

	thid = sprget_gpt(THID) >> THID_OFFSE_cpu;

	/* Create cpu topology mapping based on THID. */
	/* Multiprocessor system : Single-thread per core */
	cpuid_topo->thread_id  = -1;
	cpuid_topo->core_id    = thid & (BITS_MASK_BY_WIDTH(THID_WIDTH_cpu));
	cpuid_topo->group_id   = thid >> THID_WIDTH_cpu;

	pr_debug("CPU%u: group %d core %d thread %d thid %#016llx\n",
		 cpuid, cpuid_topo->group_id, cpuid_topo->core_id,
		 cpuid_topo->thread_id, thid);

topology_populated:
	update_siblings_masks(cpuid);
}

static void __init reset_cpu_topology(void)
{
	unsigned int cpu;

	for_each_possible_cpu(cpu) {
		struct cpu_topology *cpu_topo = &cpu_topology[cpu];

		cpu_topo->thread_id = -1;
		cpu_topo->core_id = 0;
		cpu_topo->group_id = -1;

		cpumask_clear(&cpu_topo->core_map);
		cpumask_set_cpu(cpu, &cpu_topo->core_map);
		cpumask_clear(&cpu_topo->thread_map);
		cpumask_set_cpu(cpu, &cpu_topo->thread_map);
	}
}

void __init init_cpu_topology(void)
{
	reset_cpu_topology();

	/*
	 * Discard anything that was parsed if we hit an error so we
	 * don't use partial information.
	 */
	if (parse_dt_topology()) {
		reset_cpu_topology();
	}
}
