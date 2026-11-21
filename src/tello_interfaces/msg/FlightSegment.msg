std_msgs/Header header
uint32 index          # running number of segment instances in this flight (inserted recenters included)
string type           # takeoff | goto | sync | excite | arena | hover | land
string name           # "<plan index>:<type>" or "recenter"
bool closed_loop      # true for goto and arena
string params_json    # the segment parameters (defaults filled), as JSON
