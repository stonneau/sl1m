import numpy as np

start = [[2., 1., 0.], [-2., 1., 0.], [-2., -1., 0.], [2., -1., 0.]]
end = [[2., 1., 0.], [4., 1., 0.], [4., -1., 0.], [2., -1., 0.]]

astart = np.array(start).T
aend = np.array(end).T

scene = [[astart], [aend]]
surfaces = [[[astart], [astart]], [[astart, aend], [astart, aend]], [[astart, aend], [astart, aend]], [[astart, aend], [astart, aend]], [[astart, aend], [astart, aend]], [[astart, aend], [astart, aend]], [[aend], [aend]]]
walk_surfaces = [[[astart]], [[astart]], [[astart, aend]], [[astart, aend]], [[astart, aend]], [[astart, aend]], [[astart, aend]], [[astart, aend]], [[astart, aend]], [[astart, aend]], [[astart, aend]], [[astart, aend]], [[aend]], [[aend]]]
