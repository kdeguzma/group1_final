Kyle Contributions:



Stephen Contributions:
Implemented the Report Survivor Server to to advertice on /report_survivor, log survivor_id, frame_id, and x/y cords. Wrote main search and rescue run file, implementing the BT Tree, the memory flags, and OneShot to patrol zones sequentially. Integrated Nav2 action client, ACML initilization. Wrote Design Explainations. Various Debugging and verifications to meet specificaiton requirements.


BT Design Explainations:

**Memory Flags:
Memory = True is used when you want the Behavior Tree node to remember which child was previously running or succeeded across ticks. This makes sense for the Patrol Sequencer and the Survivor Found Sequencer. The Patrol Sequence has to remember the navigate to zone process is running due to the long duration, rather than constantly checking "zones visited?" and sending the same Nav2 goal point. The Survivor Found Sequencer has to remember which node is being operated at each tick so that it doesnt constantly send service calls.

Memory = False is used when the Behavior Tree node should reevalauate the status of child nodes for each tick. This makes sense for the Root Selector and the Handle Detection Selector. The Root Selector should reevaluate the underlying conditions/actions and not get "stuck" on child node. The Handle Detection Selector needs to evaluate at each zone if there is a survivor at each zone, rather then remember the status from a previous zone.

**OneShot
The OneShot Decorator is used on the NavigateToBase action to prevent the root selector from constantly sending a goal to Nav2. This way the robot visits each zone, checks for survivors, then returns to base and stays there. The OneShot decorator prevents the Behavior Tree from resending the same Nav2 goal repeatedly.
