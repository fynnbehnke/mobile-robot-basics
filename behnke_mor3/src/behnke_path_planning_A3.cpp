#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>

/*
Erstellen eines Structs für eine Zelle.
Jede Zelle hat:
- eine ID.
- eine x,y Koordinate welche die Position im Grid angibt.
- eine Occupancy ob sie ein Obstacle (100), unbekannt (-1) oder befahrbar (0) ist.
- eine Position welche das Äquivalent zur Position im Grid ist nur in Metern auf die Karte bezogen.
- einen Boolean ob die Zelle bereits besucht wurde beim Grid durchgehen des Algorithmus.
- eine Parent_Cell die dazu dient den gesamten Pfad am Ende wiederzugeben
*/

struct Cell{
    int id;
    int x;
    int y;
    int occ;
    geometry_msgs::Pose2D coord;
    bool visited;
    struct Cell* parent_cell;
};

// Erstellen der Klasse die sich um alles kümmert.
class BreadthSearchPubSub{

private:
    
    // Festlegen der Start und Zielposition nach Angabe der Übung (mr18b070)
    double start_x = 0.6;
    double start_y = 0.6;

    double goal_x = 2.9;
    double goal_y = 3.9;

    // Deklarieren der notwendigen ROS Variablen
    ros::NodeHandle nh;
    ros::Publisher path_pub;
    ros::Subscriber map_sub;

    nav_msgs::OccupancyGrid occmap;
    nav_msgs::Path path;

    // Grid bestehend aus Cell structs in der größe der Karte (wurde beim Mappen auf 5x5m verkleinert, da sonst 4000x4000 Zellen; viel zu viele sind)    
    Cell mapgrid[96][96];

public:
    BreadthSearchPubSub(){
        // Initialisieren des OccupancyGrid subscribers und des Pfad-Publishers
        path_pub = nh.advertise<nav_msgs::Path>("/BreadthSearchPath", 1, true);
        map_sub = nh.subscribe("/map", 1, &BreadthSearchPubSub::map_cb, this);
    }

    // Breitensuche Algorithmus der die eingelesene Karte nach der Zielposition durchgeht.
    void breadthSearch(){
        
        // Berechnung der Koordinaten im grad für die Start- und Zielposition
        int start_cell_x = (start_x - occmap.info.origin.position.y)/occmap.info.resolution;
        int start_cell_y = (start_y - occmap.info.origin.position.x)/occmap.info.resolution;

        int goal_cell_x = (goal_x - occmap.info.origin.position.y)/occmap.info.resolution;
        int goal_cell_y = (goal_y - occmap.info.origin.position.x)/occmap.info.resolution;

        // Ausgangszelle ist die Start cell daher wird diese auf besucht gesetzt und sich selbst als parent zugewiesen.
        mapgrid[start_cell_x][start_cell_y].visited = true;
        mapgrid[start_cell_x][start_cell_y].parent_cell = &mapgrid[start_cell_x][start_cell_y];

        // Deklarieren einer Liste für den Pfad und die zu durchsuchenden Zellen.
        std::list <struct Cell> celllist;
        std::list <struct Cell> pathlist;

        // Hier werden der start und goal cell die  KWerte anhand der Werte an den dazugehörigenoordinaten im Grid zugeteilt.
        Cell start_cell = mapgrid[start_cell_x][start_cell_y];
        Cell goal_cell = mapgrid[goal_cell_x][goal_cell_y];

        // ALs nächstes wird in die celllist die Startzelle als Anfangsbedingung reingeschrieben.
        celllist.push_front(start_cell);

        // Deklarieren eines boolean als Beendigungsbedingung wenn das Ziel erreicht wurde.
        bool reached_goal = false;

        // Hier wird die aktuell betrachtete Zelle deklariert sowie die 8 anliegenden Nachabrn der Zelle
        Cell current_cell;
        Cell current_neighbours[8];

        // Solange bis celllist leer ist oder das Ziel gefunden wurde wird jetzt in der Karte gesucht.
        while(!celllist.empty()){

            // Zuteilen der current_cell die erste Zelle die sich in der Liste befindet (Zu Beginn nur die Startzelle).
            current_cell = celllist.front();

            // Zuweisen der Nachbarzellen in das Neighbours array.
            current_neighbours[0] = mapgrid[current_cell.x][current_cell.y + 1]; // Zelle drunter
            current_neighbours[1] = mapgrid[current_cell.x + 1][current_cell.y]; // Zelle rechts
            current_neighbours[2] = mapgrid[current_cell.x][current_cell.y - 1]; // Zelle drüber
            current_neighbours[3] = mapgrid[current_cell.x - 1][current_cell.y]; // Zelle links

            current_neighbours[4] = mapgrid[current_cell.x+1][current_cell.y+1]; // Zelle diagonal unten rechts
            current_neighbours[5] = mapgrid[current_cell.x+1][current_cell.y-1]; // Zelle diagonal oben rechts
            current_neighbours[6] = mapgrid[current_cell.x-1][current_cell.y-1]; // Zelle diagonal oben links
            current_neighbours[7] = mapgrid[current_cell.x-1][current_cell.y+1]; // Zelle diagonal unten links


            // Jetzt werden für jede Zelle die Nachbarn durchgegangen.
            for(int i = 0; i < 8; i++){
                
                // Wenn der Nachbar noch nicht besucht wurde und befahrbar ist...
                if(current_neighbours[i].occ == 0 && current_neighbours[i].visited == false){
                    // ...wird geschaut ob der Nachbar das Ziel ist.
                    if(current_neighbours[i].x == goal_cell.x && current_neighbours[i].y == goal_cell.y){
                        // Wenn ja dann wurde das Ziel erreicht und dem Nachbar wird die aktuelle Zelle als Parent zugeordnet.

                        reached_goal = true;
                        current_neighbours[i].visited = true;
                        current_neighbours[i].parent_cell = &current_cell;
                        mapgrid[current_neighbours[i].x][current_neighbours[i].y].visited = true;
                        mapgrid[current_neighbours[i].x][current_neighbours[i].y].parent_cell= &mapgrid[current_cell.x][current_cell.y];

                        break;
                    }
                    
                    // Wenn nicht dann wurde die Zelle besucht und dem Nachbarn wird die aktuelle Zelle als Parent zugordnet.
                    // Der Nachbar wird dann in die lIste geschrieben und als im nächsten durchlauf als aktuelle Zelle verwendet.
                    current_neighbours[i].visited = true;
                    current_neighbours[i].parent_cell = &current_cell;

                    celllist.push_back(current_neighbours[i]);

                    mapgrid[current_neighbours[i].x][current_neighbours[i].y].visited = true;
                    mapgrid[current_neighbours[i].x][current_neighbours[i].y].parent_cell = &mapgrid[current_cell.x][current_cell.y];

                }
            }

            // Wenn das Ziel erreicht wurde, wird die while-Schleife verlassen.
            if(reached_goal == true){
                break;
            }

            celllist.pop_front();

        }
		
        current_cell = mapgrid[goal_cell_x][goal_cell_y];

        path.header.frame_id = "/map";

        // Hier wird von der Zielzelle nun rückführend jeder Parent (die den gefundenen Pfad finden) in die Pfadliste gespeichert.
        while(1){

            pathlist.push_front(current_cell);

            // Wenn die Startzelle erreicht wurde wird die while-Schleife verlassen.
            if(current_cell.id == start_cell.id)
            {
                break;
            }
            current_cell = *current_cell.parent_cell;
        }

        // deklarieren eines PoseStamped arrays in dem schlussendlich die Pfadpositionen gespeichert werden.
        geometry_msgs::PoseStamped path_pose[pathlist.size()];

        int number = pathlist.size();
        
        // Für jede Pose in der Pfadliste wird eine Posestamped erstellt und im Pfad hinten eigereiht, sowie aus der Pfadliste geschmissen.
        for(int i = 0; i < number; i++)
        {
            current_cell = pathlist.front();
            path_pose[i].pose.position.x = current_cell.coord.x;
            path_pose[i].pose.position.y = current_cell.coord.y;
            path_pose[i].pose.orientation.w = 1;

            path.poses.push_back(path_pose[i]);
            pathlist.pop_front();
        }
        
        // Publishen des Pfades
        path_pub.publish(path);

        ROS_INFO_STREAM("Path created and published!");

    }

    void map_cb(const nav_msgs::OccupancyGrid::ConstPtr& map_msg){

        // Daten aus dem /map Topic werden in ein Occupancygrid zwischengespeichert.
        occmap.header = map_msg->header;
        occmap.info = map_msg->info;
        occmap.data = map_msg->data;

        int counter = 0;

        /*
        Für jede Zelle im Occupancygrid wird eine Zelle im Map Array beschrieben und die Daten, Occupancy,
        Zellkoordinaten, das Äquivalten in Metern und ob die Zelle bereits besucht wurde oder nicht, zugeteilt.
        */

        for(int w = 0; w < occmap.info.width; w++){
            for(int h = 0; h < occmap.info.height; h++){
                mapgrid[w][h].id = counter;
                mapgrid[w][h].x = w;
                mapgrid[w][h].y = h;
                mapgrid[w][h].visited = false;
                mapgrid[w][h].coord.x = w * occmap.info.resolution + occmap.info.origin.position.x;
                mapgrid[w][h].coord.y = h * occmap.info.resolution + occmap.info.origin.position.y;
                mapgrid[w][h].occ = occmap.data[w + occmap.info.width * h];
                counter++;
            }
        }


        // Hier wird noch einmal jedes Obstacle durchgegangen und um 4 Zellen vergrößert, damit der Pfad nicht direkt an der Wand entlang geplant wird.
        for(int f = 0; f < 5; f++){
            for(int i = 2; i < occmap.info.width-2; i++){
                for(int j = occmap.info.height-2; j > 2; j--){
                    // Für jede aktuelle Zelle wird sich die nächste Zelle angeschaut.
                    for(int next_i = 0; next_i <= 1; next_i++){
                        for(int next_j = -1; next_j <= 0; next_j++){
                            // Wenn nächste Cell ein Obstacle ist dann wird die aktuelle auch als Obstacle gesetzt.
                            if(mapgrid[i + next_i][j + next_j].occ == 100)
                                mapgrid[i][j].occ = 100;
                        }
                    }
                }
            }
        }

        ROS_INFO_STREAM("Map read was successful!");
        breadthSearch();
    }

};


int main(int argc, char** argv){

    ros::init(argc, argv, "path_pub_node");

    BreadthSearchPubSub PathPlanningObject;

    ros::spin();

    return 666; // Copyright by Fynn Behnke - mr18b070 😈
}