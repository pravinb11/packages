std_msgs/Header header
string context_name              # Which search context found it
string found_class               # "door", "stairs", etc.
geometry_msgs/PoseStamped pose   # Target pose in map frame
float64 confidence               # 0.0 to 1.0
float64 distance                 # meters from robot when detected
float64 reachability_score       # How easy to reach (0-1)