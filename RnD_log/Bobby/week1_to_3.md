#Week 1 to Week 3

##1st Feb ~ 15th Feb

ROS Nodes:

- [x] Marker Detector for ArUco

- [x] API for UAV control

- [x] Onboard SDK

- [x] Guidance SDK

- [x] PID Controller

Mechanical:

- [ ] Drone Assembling

- [ ] Manipulator Design

###Troubleshooting

While sending control commands, make sure you know which reference frame they refer to!

e.g. Body frame or Global frame (North-East-Ground Frame)

- A recommended flow control mechanism (like a finite-state machine)

```
/* Define all possible states */
typedef enum {

    INIT     ,
    TAKEOFF  ,
    STAND_BY ,
    LANDING  ,
    
} MISSION_STATUS;
```

```
MISSION_STATUS current_mission = INIT;

void mission_run() {

    switch(current_mission) {

        case INIT:
        
            //do something
        
            current_mission = TAKEOFF; // Could be changed to any other state
            break;
    
        case TAKEOFF:
    
            //do something else
        
            current_mission = STAND_BY;
            break;

        case STAND_BY:
    
            //whatever you like XD
        
            //what's next?
        
            if(condition_A_satisfied)
             
                //for example, could be a condition check for whether the mission is accomplished
                current_mission = LANDING;
            else
                current_mission = STAND_BY; //continue on doing whatever haven't been done yet
            break;
        
        case LANDING:
    
            //so...landing?
        
            current_mission = LANDING;
            break;
            
    }

}

int main() {

    //Initialization...
    
    //Loop
    while(1) {
    
        mission_run()
    }

}
```

- The new drone still haven't arrived yet...

- Keep updating! :octocat: