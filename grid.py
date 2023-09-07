class Grid():
    def __init__(self,grid_dim,cell_size) -> None:
        lx = abs(grid_dim[0]-grid_dim[2])
        ly = abs(grid_dim[1]-grid_dim[3])
        self.cell_size = cell_size
        self.nx = int(lx/cell_size)
        self.ny = int(ly/cell_size)

    def make_grid(self):
        grid = [[0 for i in range(self.nx)] for j in range(self.ny)]
        
        return grid

