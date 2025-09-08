std_msgs/Header header
bool is_monitoring               # Currently active
string active_context            # Current context name
int32 candidates_tracked         # Number of potential targets
float64 best_candidate_score     # Current best score (0-1)
float64 monitoring_duration      # How long we've been monitoring