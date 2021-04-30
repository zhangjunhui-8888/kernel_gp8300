#ifndef __ASM_TOPOLOGY_H
#define __ASM_TOPOLOGY_H

#ifdef CONFIG_NUMA

#define cpu_to_node(cpu)        ((void)(cpu),0)
#define parent_node(node)       ((void)(node),0)

#define cpumask_of_node(node)   ((void)node, cpu_online_mask)

#define pcibus_to_node(bus)     ((void)(bus), -1)
#define cpumask_of_pcibus(bus)  (pcibus_to_node(bus) == -1 ? \
                                        cpu_all_mask : \
                                        cpumask_of_node(pcibus_to_node(bus)))

#endif

#ifdef CONFIG_SMP

#include <linux/cpumask.h>

struct cpu_topology {
	int thread_id;
	int core_id;
	int group_id;
	cpumask_t thread_map;
	cpumask_t core_map;
};

extern struct cpu_topology cpu_topology[NR_CPUS];

#define topology_physical_package_id(cpu)       (cpu_topology[cpu].group_id)
#define topology_core_id(cpu)                   (cpu_topology[cpu].core_id)
#define topology_core_cpumask(cpu)              (&cpu_topology[cpu].core_map)
#define topology_thread_cpumask(cpu)            (&cpu_topology[cpu].thread_map)

void init_cpu_topology(void);
void store_cpu_topology(unsigned int cpuid);
const struct cpumask *cpu_coregroup_mask(int cpu);

#else

static inline void init_cpu_topology(void) { }
static inline void store_cpu_topology(unsigned int cpuid) { }

#endif

#include <asm-generic/topology.h>

#endif /* _ASM_TOPOLOGY_H */
