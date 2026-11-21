std_msgs/Header header
uint8 DISCONNECTED=0
uint8 LANDED=1
uint8 TAKING_OFF=2
uint8 FLYING=3
uint8 LANDING=4
uint8 EMERGENCY=5
uint8 state
string backend        # sdk | sim | fake
bool link_ok
bool hold             # /tello/hold active
float32 battery       # %, NaN if unknown
string cmd_source     # none | runner | hold | safety
float32 mocap_age     # s since the last valid Vicon pose, by arrival time (inf if none)
float32 mocap_rate    # Hz, valid Vicon poses over the last second
bool recording        # the recorder node tello_recorder is subscribed to /tello/cmd_sent
