import numpy as np


            
from trajoptpy.cart_traj_init import shortest_paths

def paths_thru_grid(x, start_inds, end_inds):
    ncosts_nk = 100 * (1-x)
    T = 100*(1 - (np.eye(x.shape[1]) + np.eye(x.shape[1],k=-1) + np.eye(x.shape[1],k=1)))
    ecosts_nkk = [T for _ in xrange(len(x)-1)]
    return [path for (path,cost) in zip(*shortest_paths(ncosts_nk, ecosts_nkk)) if cost < 100]


def test_path_thru_graph():
    grid = np.array(
        [[1,1,1,1,1],
         [0,1,1,1,0],
         [1,0,1,0,1],
         [0,1,0,1,0],
         [0,0,1,0,1],
         [0,0,1,1,1]])
    paths = paths_thru_grid(grid, range(5), range(5))
    assert (np.asarray(paths) == np.array([[0,1,0,1,2,2], [0,1,0,1,2,3], [0,1,2,3,4,4]])).all()
    
if __name__ == "__main__":
    test_path_thru_graph()