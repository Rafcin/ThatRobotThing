package com.projecttango.examples.java.pointcloud.MapViewTools;

import android.util.Log;

/**
 * Created by rafaelszuminski on 4/23/17.
 */

public class MapInfo{

    public int [][] grid;
    public int startColumn = 0;
    public int startRow = 0;
    //Grid Size instanciated in onCreate in MainActivity

    public void setGrid(int[][] grid) {
        this.grid = grid;
    }

    public int[][] getGrid() {
        return grid;
    }

    public void setCurrentCell(int gridVal, int column, int row){
        grid[column][row] = gridVal;
        Log.d("GridCell"," -- "+gridVal+" -- "+grid[0][0]);
    }

    public void startingPos(int column, int row){
        startRow = row;
        startColumn = column;
    }

}
