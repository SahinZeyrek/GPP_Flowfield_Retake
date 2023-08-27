# Flowfield pathfinding
## _Research Project GPP_


In video games we move objects from point a to point b quite often,
which can be realised through pathfinding algorithms such as:
 AStar,DStar etc.
 These are great pathfinding algorithms for a few amount of objects.
 But what if we need to move objects on a terrain that changes a lot 
 and with a lot of objects in the quickest way possible?
 
 ## Introducing : Flowfield
 
 

> Primarly used in RTS games, the flowfield is a common way to move a large amount of objects to a singular goal point
> in an efficient and quick way. 

The Flowfield consists of
- Grid
- Heat map/Cost field
- Vector field
 ## Grid
Like most pathfinding algorithms, it consists of a grid. The grid itself consists of
nodes or cells where each cell or node has a certain cost.
Depending on what the type of cell it is the cost could increase.
For example: Dirt would be harder to traverse than grass.
You could also have impassable terrain like a wall or water.
Each cell will hold a value between 1-255.
1 being the best cost and 255 being an impassable terrain.
Anything between 2-254 should be avoided if possible.


## 1: Heat map/Cost field
The heat map or the cost field would be the very first step after you'd have created your grid,
attributing the distance from the cell towards the goal point.
The algorithm itself is similair to AStar.
We keep track of our cells by using an open list and closed list. Any visited cell gets put into the closed list.
We would go over each cell in the grid, give it a certain cost depending on the type of terrain
and start calculating the heatmap starting from our goal point.
After that, you would add it to the open list and start iterating over the list as long its not empty.
Then we'd take the cell with the least amount of cost as our current node
and it to our closed list.
For each neighbour of the current node in the list you would first check if its a special terrain.
If its in impassable terrain it would receive the cost of 255 and we'd immediately go the next cell in the open list. 
If its anything else you could give it a 
cost between 2-254. It's up to the developer to decide.

Then we check if this node is already in the closed list.
If thats the case:
We apply the following rule:
 > If our current node plus the distance between each cell is smaller than our visited node,
 then that means that we have found a cheaper route and we set the cost of our visited node 
 to the sum of the current nodes cost plus the distance between each cell
 
 > Repeat this logic with the open list as well.

If its not in the open list or closed list, we create a new cost
equaling to our current cell plus the distance between each cell
and add it to our open list.
## Heat map
![N|Solid](https://i.imgur.com/SaP7DGc.png)
## 2: Vector field
 
Now that we have our costs/distance to goal for each cell; It's time to calculate our vectors.
To do this we will be using a process called Kernel Convolution.
Simply put its a function that would take a cell and iterate over all its neighbours and take the average
of all the neigbouring costs.
After it would add up all the costs of the cell and take the average of it.
This results in a vector being created pointing towards the goal, following the cost field.

There is a slight exception to this rule where our neighbours could be an impassable terrain
or our current cell would be at the edge of the grid.
If this is the case then you would create a vector using the smallest cost of your neighbouring cells.

![N|Solid](https://i.imgur.com/NQ3P3rJ.png)
## 3: Applying Vectors to objects
As a final step our object that requires moving would get the direction of the current cell it would find itself in.


![N|Solid](https://i.imgur.com/VGpwaNN.gif)


# Conclusion
This was my second attempt at making a flow field and I'm proud that I've been able to do it this time.
I've improved quite a bit ever since and I'm looking forward to improving more in the fields I'm interested in.
It has been quite the experience and challenge.

