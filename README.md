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
&emsp; - Increment joint angles in both directions to get 16 cases\
&emsp; - Iterate (sample) new joint angles (all 16 cases)\
&emsp; &emsp; - finding COG location function for each case\
&emsp; &emsp; - Cost function (current COG - target COG)^2\
&emsp; - Select lowest cost and do gradient descent
