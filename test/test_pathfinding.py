from src.mazeMapping import Map
from src.pathfinding_engine import a_star, visualize_path

if __name__ == "__main__":
        #Create a map with specified parameters
        start_node = (0,0)
        goal_node = (49,49)
        map=Map(width=50,height=50,density_rate=0.3,hardness_rate=0.5,start=start_node,goal=goal_node,seed=42)
        #Assign the generated map to a variable
        grid = map.generate_map()
        #A star implementation to find the route from start to goal and then show it
        route = a_star(cost_map=grid, start=start_node, goal=goal_node)
        show_route = visualize_path(cost_map=grid, path=route, start=start_node, goal=goal_node)
        
        