# Motion_Planning_Group12

## Group Member
&emsp; - Kevin Nelson\
&emsp; - Sam Spencer\
&emsp; - Jordan Raver\
&emsp; - Lin Ouyang\
&emsp; - Nathan Fuller\
&emsp; - Zi Wang

## Algorithm Flowchart
![alt text](https://github.com/Zi-23/Motion_Planning_Group12/blob/895ce2a9f00716a7a2e42dff2a2df886daa95509/AlgorithmFlowchart.png?raw=true)

## Gradient Descent
&emsp; - Increment joint angles in both directions to get 16 cases.\
&emsp; - Iterate (sample) new joint angles (all 16 cases).\
&emsp; &emsp; - finding COG location function for each case.\
&emsp; &emsp; - Plug each new COG into cost function (current COG - target COG)^2\
&emsp; &emsp; - Select lowest cost and add to minimum cost function array.\
&emsp; - Once the current COG is within a range around the target COG, plot path and move the robot.

## Results
### Threshold = 1
![alt text](https://github.com/Zi-23/Motion_Planning_Group12/blob/ea71bf7befedcb01c6833589b85e1c8d2b6aa8ee/COG_Plot_Threshold_1.png?raw=true)

### Threshold = 0.01
![alt text](https://github.com/Zi-23/Motion_Planning_Group12/blob/ea71bf7befedcb01c6833589b85e1c8d2b6aa8ee/COG_Plot_Threshold_0_01.png?raw=true)
