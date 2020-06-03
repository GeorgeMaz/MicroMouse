// Micro Mouse Project Group 6.
#include <stdio.h>
#include <stdlib.h>

// Global Variables need to be delcared here

int direction = 0; // Have to set up a variable which tells us which direction the mouse will be pointing in
// 0 - North  // 1 - East                                                          
// 2 - South  // 3 - West

int maze[9][9]; // Array which stores the path count. Same as the maze variable in the simulation.                       
int wall[4];    // Same as directions 0 - 3; however when any of the cells    
                // are 1 this means there is a wall. Basically converting the 
                // analogue signal to a digital one by using a threshold 
                // This is the same as current_Walls[4] in the simulation code     

int *xptr; int *yptr; // Pointers maze co-oridinates (use pointers to update i and j) 
int i = 0; int j = 0; // actual values of maze co-ord's

// Variables for the sensors 
int front, left, rear, right;

// How many of these actually need to be global?!
int path = 0; // The path count
int explored = 0;
int intialise_move = 0; 
int wantedDirection;
int walls[9][9][4];
int dead = 0;
int n = 0;
int map_Paths[4];
int total_Paths;
int centre = 0;
int shortest_Route = 99;
int count = 0;
int centre_Trigger = 0;
int done = 0;
int junction_Dead = 0;

int junctions[2][50];    //position of junction array  

// Variables For the right encoder; 

const byte encoder0pinA = 2;                  //A pin -> the interrupt pin 0
const byte encoder0pinB = 4;                  //B pin -> the digital pin 4
byte encoder0PinALast;
int duration;                                 //the number of the pulses
boolean Direction;                            //the rotation direction 


//-------------------------------------------------------------------------

// Setup is executed before void loop(); this is to
// set up the GPIO's, set them as inputs/outputs etc.
// Its also used to initlise some variables such as the maze.

void setup() {

  // Motor Shield Pins Taken From the Arduino Website
  //Function  pins per Ch. A  pins per Ch. B
  //Direction           D12             D13
  //PWM                 D3              D11
  //Brake               D9              D8
  //Current Sensing     A0              A1

  // Motor A - Left
  pinMode(12, OUTPUT); // Direction
  pinMode(3, OUTPUT); // PWM - Duty Cycle changes the speed
  pinMode(9, OUTPUT); // Break
  pinMode(A0, INPUT); // Current Sensing

  //Motor B - Right
  pinMode(13, OUTPUT); // Direction
  pinMode(11, OUTPUT); // PWM - Duty Cycle changes the speed
  pinMode(8, OUTPUT); // Break
  pinMode(A1, INPUT); // Current Sensing

  // Define sensor inputs here
  // These are anlogue input here
  pinMode(A2, INPUT); // Front
  pinMode(A3 , INPUT); // Left
  pinMode(A4 , INPUT); // Right
  pinMode(A5 , INPUT); // Rear

  // Global Variables that i'm putting here so it's easy to adjust stuff
  
  // Duty Cycle & Hence Speed
  #define duty 150 //  AnalogWrite(Pin, Dutycycle), 0 - 255; 127 is 50%
  #define rtpulse 4 // enoder pulse count for right turn 
  #define ltpulse 3 // enoder pulse count for right turn 
  #define fpulse 14 // forward pulse duration

  // For the Analogue to digital threshold values
  #define adfront 320
  #define adrear 300
  #define adleft 315
  #define adright 300

  // For the forward centering control 
  // uses the forward threshold value 
  #define cleft 265
  #define cright 250

  
 // This is the value that the duty cycle is adjusted by.  
  #define cduty 30 


  // This function initialises the maze with 99's everywhere
  for (int k = 0; k < 9; k++) {
    for (int p = 0; p < 9; p++) {
      maze[k][p] = -1;
    }
  }
 // Initilising the main maze with all 0's 
  for(int i=0; i<9; i++){
    for(int j = 0; j < 9; j++){ 
      for(int k = 0; k < 4; k++){
            walls[i][j][k] = 0;
        }
      }
    }

  // Serial Comms;
  // This will just help with Debugging, especially being able to see the direction
  Serial.begin(9600);

  EncoderInit(); // for the encoder to work

  // Dealing with the maze co-ordinates
  xptr = &i;
  yptr = &j; // Storiniisg address of i and j within pointers


}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------

void loop() {

   mapper();
   //print_maze(maze);
//   
//    int val = analogRead(A4);
//     int val1 = analogRead(A3);
//     int val2 = analogRead(A2);
//     //int val3 = analogRead(A4); 
//     Serial.print(val);
//     Serial.print("\t");
//     Serial.print(val1);
//     Serial.print("\t");
//     Serial.println(val2);
     

   //turn_left();
//
//  movemulti(6);  
//  turn_right();
//   delay(3000);
//   turn_right();
   


}
//-------------------------------------------------------------------
// This function move forward x number of blocks
void movemulti(int x){
    for(int j = 0; j < x; j++){
    move(wall, xptr, yptr);

    delay(2000);
   }
}
//--------------------------------------------------------
void mapper(){

  //while to loop through function as long as there are unmarked paths
  while(explored == 0){
   Serial.print("\n");

   
   scan(wall, xptr, yptr,walls);
   
   fortesting();
   printwalls(wall);
   print_maze(maze);


// finds available paths and returns wanted direction
   wantedDirection = find_Path(xptr, yptr, wall, maze);
   fortesting();
   delay(2000);
  
   //changes direction to wanted direction
   change_Direction();

  //redefines walls after changing direction
  scan(wall, xptr, yptr, walls);
  delay(1000);
  printwalls(wall);
  
  //checks can move forward and returns initialise move
   intialise_move = forward_Check(wall , xptr, yptr, intialise_move, maze);
   delay(2000);
   fortesting();
   printwalls(wall);
   print_maze(maze);
   delay(2000);

   update_maze(xptr, yptr, maze);

    if(intialise_move == 1){
     move(wall, xptr, yptr);
   }

  stop(wall, xptr, yptr, maze);
  }  
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------
void Physical_move(){
  // The forward command is ran until the mouse has moved forward one square
  // This is dependant on feedback from the encoders 
  // upCor(); has to be called after forward, as to update the co-ordinates.
  // these could almost be concatinated into one move() function.

  while(duration < fpulse){
    forward();
   }
  duration = 0; // reset the duration 
  //upCor(xptr, yptr);
  //update_maze(xptr, yptr, maze);
  }
  

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------
void scan(int matrix[4], int *x, int *y, int walls[9][9][4]){
  // The scan(); Function fills a matrix[4] of the information of whether there is a wall there or not. 
  // It works by thresholding the analogue value to turn it into a digital output. 
  // it then stores this in a larger matrix which represents the maze. 
  
  front = analogRead(A2);
  left = analogRead(A3);
  right = analogRead(A4);
  rear = analogRead(A5);

  // matrix[0] = North
  // matrix[1] = East 
  // matrix[2] = South 
  // matrix[3] = West 

 // Conversion from Analogue to digital 
  int dfront = 0; 
  int dright = 0;
  int drear = 0; 
  int dleft = 0;
  
  //Im guessing because of the voltage loss between the boards the sensors are returning slightly different values
  // This is also quite heavily dependant on ambient light conditions.
  if(front < adfront){
    dfront = 1;
  }
  if(right < adright){
    dright = 1;
  }
  if(rear < adrear){
    drear = 1;
  }
  if(left < adleft){
    dleft = 1;
  }

  // This next section assigns left, right, front and rear to the actual 
  // physical direction. It then assigns 
  if(direction == 0){
     matrix[0] = dfront;
     matrix[1] = dright;
     matrix[2] = drear;
     matrix[3] = dleft; 

     walls[*x][*y][0] = dfront;
     walls[*x][*y][1] = dright;
     walls[*x][*y][2] = drear;
     walls[*x][*y][3] = dleft;
     
  }else if(direction == 1){
     matrix[0] = dleft;
     matrix[1] = dfront; 
     matrix[2] = dright;
     matrix[3] = drear;

     walls[*x][*y][0] = dleft;
     walls[*x][*y][1] = dfront;
     walls[*x][*y][2] = dright;
     walls[*x][*y][3] = drear;
     
  }else if(direction == 2){    
     matrix[0] = drear;
     matrix[1] = dleft;
     matrix[2] = dfront;
     matrix[3] = dright;   

     walls[*x][*y][0] = drear;
     walls[*x][*y][1] = dleft;
     walls[*x][*y][2] = dfront;
     walls[*x][*y][3] = dright;
     
  }else if(direction == 3){    
     matrix[0] = dright;
     matrix[1] = drear;
     matrix[2] = dleft;
     matrix[3] = dfront; 

     walls[*x][*y][0] = dright;
     walls[*x][*y][1] = drear;
     walls[*x][*y][2] = dleft;
     walls[*x][*y][3] = dfront;
  }
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------
// This is just going to be used for testing really however, 
// it prints to the serial moniter as to which direction the walls are in. 
void printwalls(int matrix[4]) {
    Serial.print("North:");
    Serial.print(matrix[0]);
    Serial.print("\n");
    
    Serial.print("East:");
    Serial.print(matrix[1]);
    Serial.print("\n");
    
    Serial.print("South:");
    Serial.print(matrix[2]);
    Serial.print("\n");
    
    Serial.print("West:");
    Serial.print(matrix[3]);
    Serial.print("\n\n");
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------
void forward() {
  
  front = analogRead(A2);
  left = analogRead(A3);
  right = analogRead(A4);
  
  digitalWrite(9, LOW); // Un-Break Left
  digitalWrite(8, LOW); // Un-Break Right
  
  //-----------------------------------------------------------------
  //-----------------------------------------------------------------
  // This section of code below reduces the PWM to each of the motors as it gets too close to 
  // either the left or right wall; it could be worth implementing a PID controller here.
  // This first if statment stops the mouse driving straight into a wall
     if((front < adfront) || (front < adfront && right < cright) ||(front < adfront && left < cleft)){
           digitalWrite(9, HIGH);             // Break Left
           digitalWrite(8, HIGH);}            // Break Right
     else if(right < cright){ // If too close to the right wall (slow left)  
           analogWrite(3, (duty - cduty));} // Left motor speed reduced  
     else if(left < cleft){ // If too close to the left wall  (slow right)  
           analogWrite(11, (duty - cduty));} // Right motor speed reduced
     else{
           analogWrite(3, duty); // Left
           analogWrite(11, duty); } // Right 

  //-----------------------------------------------------------------
  //-----------------------------------------------------------------


 
  // Left  & Right wheel goes forward
  digitalWrite(12, HIGH); // Motor A Direction
  digitalWrite(13, HIGH); // Motor B Direction

  delay(10); // need this delay for accuracy 

  digitalWrite(9, HIGH); // Break Left
  digitalWrite(8, HIGH); // Break Right

  return;
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------
void upCor(int *x, int *y){
    // These lines below update the co-ordinates (Hence, UpCor) of the maze so that we know where the mouse is pointing.
   // ( NB. could just use a for loop to run for one square and then update the co-oridinates)
  if (direction == 0) {
    *y += 1;
  }
  if (direction == 1) {
    *x += 1;
  }
  if (direction == 2) {
    *y -= 1;
  }
  if (direction == 3) {
    *x -= 1;
  }
      path++;
}
// --------------------------------------------------------------------------------------------------------------------------------------------------------------------
void update_maze(int *x, int*y, int maze[9][9]){
    maze[*x][*y] = path;
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------

void turn_left() { // input is direction
  // These directions could be wrong, we're not going to know until we try it.

  digitalWrite(9, LOW); // Un-Break Left
  digitalWrite(8, LOW); // Un-Break Right

  analogWrite(3, duty); // left
  analogWrite(11, duty); // Motor B Speed - right
  
  while(duration > (-1*ltpulse)){ //-1 as wheel is turning in the opposite direction
  // Left wheel goes back
  digitalWrite(12, LOW); // Motor A Direction

  //Right Wheel goes forward
  digitalWrite(13, HIGH); // Motor B Direction

    
  //Serial.print("Duration:");
  //Serial.print(duration);
  //Serial.print("\n");

  } 
  digitalWrite(9, HIGH); // Break Left
  digitalWrite(8, HIGH); // Break Right
  duration = 0;

  // Direction updater
  if (direction > 0) {
    direction -= 1;
  }
  else {
    direction += 3;
  }
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------

void turn_right() { // again x is the direction variable in this case

  digitalWrite(9, LOW); // Un-Break Left
  digitalWrite(8, LOW); // Un-Break Right

  analogWrite(3, duty); // left
  analogWrite(11, duty); // Motor B Speed - right

  while(duration < rtpulse){

   //Serial.print("Duration:");
   // Need this print atatment to work for some bizzare reason
    Serial.print(duration);
   Serial.print("\n");


      // Left wheel goes forward
  digitalWrite(12, HIGH); // Motor A Direction

     //Right Wheel goes back
  digitalWrite(13, LOW); // Motor B Direction
  
  } 
  digitalWrite(9, HIGH); // Break Left
  digitalWrite(8, HIGH); // Break Right
  duration = 0;

  // Direction updater
  if (direction == 3) {
    direction -= 3;
  }
  else {
    direction += 1;
  }
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------

// Print the maze to the serial moniter
void print_maze(int matrix[9][9]){
    for(int k = 0; k < 9; k++){
        for(int p = 0; p < 9; p++){
          Serial.print(matrix[k][p]);
        }
        Serial.print("\n");
      }
    Serial.print("\n\n");
  }
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------
void fortesting(){
  Serial.print("Direction:");
  Serial.print(direction);
  Serial.print("\n");
  Serial.print("x:");
  Serial.print(i);
  Serial.print("\n");
  Serial.print("y:");
  Serial.print(j);
  Serial.print("\n");
  Serial.print("Explored:");
  Serial.print(explored);
  Serial.print("\n");
  Serial.print("Wanted Direction:");
  Serial.print(wantedDirection);
  Serial.print("\n");
  Serial.print("Initilise Move:");
  Serial.print(intialise_move);
  
  Serial.print("\n\n");}
//-----------------------------------------------------------------------------------------------------------------------------
void change_Direction(){

  //if the no change to direction
  if(direction == wantedDirection){
  direction = direction;
  }
  //a call for turning right
  if(direction == 0 && wantedDirection == 1 || direction == 1 && wantedDirection == 2 || direction == 2 && wantedDirection == 3 || direction == 3 && wantedDirection == 0){// if statement to determine a turn right
    turn_right();
  }
  //a call for turning left
  if(direction == 0 && wantedDirection == 3 || direction == 1 && wantedDirection == 0 || direction == 2 && wantedDirection == 1 || direction == 3 && wantedDirection == 2){// if statement to determine a turn left
    turn_left();
  }

  if(direction == 0 && wantedDirection == 2 || direction == 2 && wantedDirection == 0 || direction == 1 && wantedDirection == 3 || direction == 3 && wantedDirection == 1){// if statement to determine a turn left

    turn_right();
    delay(1000);
    turn_right();
  }
}

//----------------------------------------------------------------------------------------------------------------------------------------------------

int find_Path(int *x, int *y, int current_Walls[4], int maze[9][9]){
    
    //if front sensor equals one (wall in front), change direction. Also if the cell infront doesnt have -1 then this cell has been explored so change direction
    if (current_Walls[0] == 1 || direction == 0 && maze[*x-1][*y] > -1 || direction == 1 && maze[*x][*y+1] > -1 || direction == 2 && maze[*x+1][*y] > -1 || direction == 3 && maze[*x][*y-1] > -1){ 
      
      //loop through current walls to find the a gap and makes this the wanted direction
              for(int d=0; d < 4; d++){ 
          
          // determines which direction the mouse should change to, if there is a '0' theres a gap
                    if(current_Walls[d] == 0 && d == 0 && maze[*x-1][*y] == -1){
                        wantedDirection = d;     //assigns this value to the wanted direction for movement                    
                        break;  //if this condition is met break the loop
                    }
          //repeats process to find a gap if possible 
                    if(current_Walls[d] == 0 && d == 1 && maze[*x][*y+1] == -1){ 
                        wantedDirection = d;      
                        break; }

                    if( current_Walls[d] == 0 && d == 2 && maze[*x+1][*y] == -1){
                        wantedDirection = d;        
                        break; }

                    if(current_Walls[d] == 0 && d == 3 && maze[*x][*y-1] == -1){ 
                        wantedDirection = d;               
                        break; }
                }
    }

    else{
    // if none are true then the mouse does not need to change direction
    wantedDirection = direction;
  
    }
  return wantedDirection;
}
//------------------------------------------------------------------------------------------------------
int stop(int current_walls[4], int *x, int *y, int maze[9][9]){

  int a = 0, b = 0, c = 0, d = 0;

  if(current_walls[0] == 0 && direction == 3 && maze[*x][*y-1] > 0){
    a = 1;
  }
  
  if(current_walls[0] == 0 && direction == 2 && maze[*x+1][*y] > 0){
    b = 1;
  }
  
  if(current_walls[0] == 0 && direction == 1 && maze[*x][*y+1] > 0){
    c = 1;
  }

  if (current_walls[0] == 0 && direction == 0 && maze[*x-1][*y] > 0){
    d = 1;
  }
  if(done == 1){
    explored = 3;
  
  }
}

//-------------------------------------------------------------------------------------------
int forward_Check(int current_walls[4], int *x, int *y, int initialise_Move, int maze[9][9]){
  
  //checks cell in front for a marked path and gap, North direction
  if (current_walls[0] == 0 && direction == 0 && maze[*x-1][*y] < 0){
    initialise_Move = 1;
    return initialise_Move;
 
  }
  //checks cell in front for a marked path and gap, East direction
  if(current_walls[0] == 0 && direction == 1 && maze[*x][*y+1] < 0){
    initialise_Move = 1;
    return initialise_Move;

  } 
  
  //checks cell in front for a marked path and gap, South direction
  if(current_walls[0] == 0 && direction == 2 && maze[*x+1][*y] < 0) {
    initialise_Move = 1;
    return initialise_Move;
 
  }
 
  //checks cell in front for a marked path and gap, West direction  
  if (current_walls[0] == 0 && direction == 3 && maze[*x][*y-1] < 0){
    initialise_Move = 1;
    return initialise_Move;}
  else{
    return 0;}
}

//--------------------------------------------------------------------------------------------------------
void wheelSpeed(){
  int Lstate = digitalRead(encoder0pinA);
  if((encoder0PinALast == LOW) && Lstate==HIGH){
    int val = digitalRead(encoder0pinB);
    if(val == LOW && Direction){
      Direction = false;                       //Reverse
    }
    else if(val == HIGH && !Direction){
      Direction = true;                        //Forward
    }
  }
  encoder0PinALast = Lstate;
  if(!Direction)  duration++;
  else  duration--;
}
//-------------------------------------------------------------------------------------------------
void EncoderInit(){
  Direction = true;                            //default -> Forward  
  pinMode(encoder0pinB,INPUT);  
  attachInterrupt(0, wheelSpeed, CHANGE);
}
//-------------------------------------------------------------------------------------------------
//maze_Update(xptr,yptr, maze);
void maze_Update(int *x, int *y, int maze[9][9]){

      if(explored == 3){
         maze[*x][*y] = path;  
      }

        if(done == 1){
          maze[*x][*y] = 000;
        }
        if(explored == 2 && done == 0){
          maze[*x][*y] = 0;
        }

        //Serial.print("\n path in maze_Update: %d\n", path);
        if(dead == 0 && explored != 2 && done == 0|| dead == 3 && explored != 2 && done == 0){
        maze[*x][*y] = path;
        }

        if(dead == 1 && explored != 2 && done == 0){
          maze[*x][*y] = 99;
          path--;
        }
        if(dead == 2 && explored != 2 && done == 0){
        maze[*x][*y] = 98;
        }

        if(centre == 1 && explored != 2 && done == 0){
          maze[*x][*y] = path;
        }
}
//--------------------------------------------------------------------------------

// move(walls, xptr, yptr);
void move(int matrix[4], int *x, int *y){
  // Dectecting front sensor is zero
  
  if (matrix[0] == 0){
    //determin direction and move north 
    if(direction  == 0){

      if(dead == 0 || explored == 2){
        path ++;    //add to path count
      }
      if(centre == 1 && explored != 2 || dead == 2 && explored != 2|| dead == 3 && explored != 2){
        path --;
      }
      *x = *x - 1;    //update 'x'
     maze_Update(xptr,yptr, maze);
    }
    //determin diection and move east
    if(direction == 1){
      if(dead == 0 || explored == 2){
        path ++;    //add to path count
      } 
      
      if(centre == 1 && explored != 2|| dead == 2 && explored != 2 || dead == 3 && explored != 2){
        path --;
      }
      *y = *y + 1;    //update 'y'
      maze_Update(xptr,yptr, maze);

    }
    //determin diection and move south
    if(direction  == 2) {

      if(dead == 0 || explored == 2){
        path ++;    //add to path count
      }

      if(centre == 1 && explored != 2|| dead == 2 && explored != 2|| dead == 3 && explored != 2){
        path --;
      }
      *x = *x + 1;    //update 'x'
       maze_Update(xptr,yptr, maze);

    }
    //determin diection and move west
    if (direction  == 3){
      if(dead == 0 || explored == 2){
        path ++;    //add to path count
      }

      if(centre == 1 && explored != 2 || dead == 2 && explored != 2 || dead == 3&& explored != 2){
        path --;
      }
      *y = *y - 1;    //update 'y'
      maze_Update(xptr,yptr, maze);
    }
  }
}
//---------------------------------------------------------------
//return_to_Junction(xptr, yptr, junctions)
void return_to_Junction(int *x, int *y, int junctions[2][50]){

 maze_Update(xptr,yptr, maze);   //updates map

  if(dead == 1){
  path++;
  }
  
  //while loop to return to the previous junction for both types of dead ends
  while(dead == 1 || dead == 2){
  delay(200);  
  //if statement to determine if the current position of the mouse it at the previous junction
     if(*x == junctions[n-1][0] && *y == junctions[n-1][1]){
      scan(wall, xptr, yptr,walls);
    
    //if statement for dead end type '1' and centre trigger '0' (centre hasnt been found)
      if(dead == 1 && centre_Trigger == 0){
      possible_Paths(xptr,yptr, map_Paths, wall);  //checks to see if there are any possible paths (-1)
      //if there are no possible paths the return dead == 1
    if(total_Paths == 0){
          dead = 1;   //sets dead to '1'
        }
      }
     
   //if statement for dead == 1 and centre == 1 (centre has been found)
      if(dead == 1 && centre_Trigger == 1){
    path ++;    //adds to path
      any_Unmapped(map_Paths, xptr, yptr, wall);
    //if there are no total paths
          if(total_Paths == 0){
            explored = 1; //sets explored to '1'
          }
      }
  //if statement for dead == 2
      if(dead == 2){
       any_Unmapped(map_Paths, xptr, yptr, wall);  //checks for unmapped pathways
    //if there isnt any unmapped paths
          if(total_Paths == 0){
            explored = 1; //sets explored to '1'
          }
      }
  
  //if statement for no dead end
    if (dead == 0){
       maze_Update(xptr,yptr, maze);  //updates the map
       wantedDirection = find_Path(xptr, yptr, wall, maze);   //tries to find the path and returns a wanted direction

      }
 }
  //if statement for dead == 1
   if(dead == 1){
      wantedDirection = find_Path(xptr, yptr, wall, maze);    //find wanted direction
   }
  //if statement for dead == 2
    if(dead == 2){
    //if the mouse has returned to position (0,0)
      if(x == 0 && y == 0){
        dead = 0;   //sets dead to 0
        explored = 2;   //sets explored to '2'
      }
 
    wantedDirection = find_Path(xptr, yptr, wall, maze);    //returns wanted direction from the (from_centre)
   }
      change_Direction();   //changes the direction
      scan(wall, xptr, yptr,walls);      //updates the walls
      move(wall, xptr, yptr);
  }
}
//--------------------------------------------------------------------------------------
//n might have to be a pointer that updates, i dont think i can put it into an array!!!
void at_Junction(int *x, int *y, int junctions[2][50]){
  junctions[n][0] = *x;  //saves the x into an array
  junctions[n][1] = *y;  //save the y into an array
  n ++;          //add number of junctions
}
//at_Junction(xptr,yptr,junctions)
//----------------------------------------------------------------------------------------
//dead_End_Check(xptr, yptr, walls, maze)
void dead_End_Check(int *x, int *y,int matrix[4], int maze[9][9] ){
  
   // find_Path(direction); //wantedDirection = find_Path(xptr, yptr, wall, maze); why is it here
  
  if (matrix[0] == 1 && matrix[1] == 1 && matrix[3] == 1){ //if all sensors detect a wall then there is a physical dead end
    dead = 1; //sets dead to 1
  }
  
  //if there is no unmapped cells to move into then there is a dead end of type '1'
    if(maze[*x-1][*y] > 0 && maze[*x][*y+1] > 0 && maze[*x+1][*y] > 0 && maze[*x][*y-1] > 0){
    dead = 1; //sets dead to 1

    }
  
  //if there is a cell that has a '-1' then there is an unmapped route, therefore sets the dead 0
    if(maze[*x-1][*y] < 0 || maze[*x][*y+1] < 0 || maze[*x+1][*y] < 0 || maze[*x][*y-1] < 0){
    dead = 0; //sets dead to 0
    }
}

//---------------------------------------------------

   //deletes the previous junction
   //delete_Junction(junctions);
   void delete_Junction(int junctions[2][50]){
    n--;  //minus' from the number of junctions that there is
    junctions[n][0] = -1;   //resets the junction to '-1'
    junctions[n][1] = -1;   //resets the junction to '-1'

}
//-------------------------------------------------------------------------------------
//possible_Paths(xptr,yptr, map_Paths, wall)
void possible_Paths(int *x, int *y, int map_Paths[4], int current_Walls[4]){

  scan(wall, xptr, yptr,walls);    //current walls
  //for loop fills map_Paths with '0'
  for(int i=0; i<4; i++){
    map_Paths[i] = 0;     
  }
  
  //if there is a -1 in the direction stated in the if statement
  if(maze[*x-1][*y] < 0){
    map_Paths[0] = 1;   //sets map path the '1'
  }

  //if there is a -1 in the direction stated in the if statement
  if(maze[*x][*y+1] < 0){
    map_Paths[1] = 1;   //sets map path the '1'
  }

  //if there is a -1 in the direction stated in the if statement
  if(maze[*x+1][*y] < 0){
    map_Paths[2] = 1;   //sets map path the '1'
  }

  //if there is a -1 in the direction stated in the if statement
  if(maze[*x][*y-1] < 0){
    map_Paths[3] = 1;   //sets map path the '1'
  }

  int gaps[4];
    for (int i = 0; i<4; i++){    
       gaps[i]= (current_Walls[i] - 1) * (current_Walls[i] - 1);    //defines gaps as the opposite of current walls
    }
 
  //multiplies gaps with possible paths then adds them together to give total paths
    total_Paths =(map_Paths[0] * gaps[0]) + (map_Paths[1] * gaps[1]) + (map_Paths[2] * gaps[2]) + (map_Paths[3] * gaps[3]); 
//    printf("map_Paths[0]: %d\n gaps[0]: %d\n map_Paths[1]: %d\n gaps[1]: %d\n map_Paths[2]: %d\n gaps[2]: %d\n map_Paths[3]: %d\n gaps[3]: %d\n ", map_Paths[0], gaps[0], map_Paths[1], gaps[1], map_Paths[2], gaps[2], map_Paths[3], gaps[3]);

  //if total paths is more than '0', then dont mark as a dead end
    if(total_Paths > 0){
      dead = 0;   //sets dead to 0
    }

  //if there is no total_paths then delete the junction
    if(total_Paths == 0){
      delete_Junction(junctions);    //calls delete junction
      dead = 1;             //sets dead end to 1
    }
}
//-----------------------------------------------------------------------------------------------------------------------------
//function determines if the current position 
//found_Centre(xptr, yptr); 
void found_Centre(int *x, int *y){
  
  //if x and y is equal to 4
    if(*x == 4 && *y == 4){
        centre = 1;   //sets centre to '1'
        centre_Trigger = 1;   //sets centre_Trigger to '1'
        explored = 1;   //sets explored to '1'
        shortest_Route = path;  //saves the shortest_Route to the current path
  }
}
//------------------------------------------------------------------------------------------------------
//this function finds the wantedDirection to return from the centre of the maze
//from_Centre(xptr,yptr, maze);
int from_Centre(int *x, int *y, int maze[9][9]){
  if(*x == 0 && *y == 0){
    explored = 2;   //sets explored to '2'
  }
  int route = path - 1;   //defines route to 'path - 1' 
  //if the value in the cell stated if equal to route then set the direction to the correct value
    if(maze[*x-1][*y] == route){
      wantedDirection = 0;     //sets wanted direction to '0'
    }
  //if the value in the cell stated if equal to route then set the direction to the correct value
    if(maze[*x][*y+1] == route){
      wantedDirection = 1;     //sets wanted direction to '1'
    }

    //if the value in the cell stated if equal to route then set the direction to the correct value
    if(maze[*x+1][*y] == route){
      wantedDirection = 2;     //sets wanted direction to '2'
    }

  //if the value in the cell stated if equal to route then set the direction to the correct value
    if(maze[*x][*y-1] == route){
      wantedDirection = 3;     //sets wanted direction to '3'
    }
    return wantedDirection;
}
//----------------------------------------------------------------------------------------------------------
//this function finds any unmapped paths from the current junction
//any_Unmapped(map_Paths, xptr, yptr, wall);
void any_Unmapped(int map_Paths[4], int *x, int *y, int current_Walls[4]){

  for(int i=0; i<4; i++){
    map_Paths[i] = 0;     
  }

  //if there is a -1 in the direction stated in the if statement
  if(maze[*x-1][*y] < 0){
    map_Paths[0] = 1;   //sets map path the '1'
  }
  
  //if there is a -1 in the direction stated in the if statement
  if(maze[*x][*y+1] < 0){
    map_Paths[1] = 1;   //sets map path the '1'
  }
  
  //if there is a -1 in the direction stated in the if statement
    if(maze[*x+1][*y] < 0){
    map_Paths[2] = 1;   //sets map path the '1'
  }
  
  //if there is a -1 in the direction stated in the if statement
    if(maze[*x][*y-1] < 0){
    map_Paths[3] = 1;   //sets map path the '1'
  }

  int gaps[4];
  for (int i = 0; i<4; i++){
    gaps[i]= (current_Walls[i] - 1) * (current_Walls[i] - 1);     //defines gaps as the opposite of current walls
//    printf("\ncurrent_Walls[%d]: %d\n", i, current_Walls[i]);
  }
 
 
  //multiplies gaps with possible paths then adds them together to give total paths
  total_Paths =(map_Paths[0] * gaps[0]) + (map_Paths[1] * gaps[1]) + (map_Paths[2] * gaps[2]) + (map_Paths[3] * gaps[3]); 

  //if total paths == 0 then the delete the current junction
    if(total_Paths == 0){
      delete_Junction(junctions);    //delete junction is called
      centre = 1;}       //sets centre to '1'
  
    if(total_Paths > 0){
        explored = 0;  }   //set explored to 0
}
//-------------------------------------------------------------------------------------------------------------

//this function returns the mouse to the start point
// return_Centre(xptr,yptr);
void return_Centre(int *x, int *y){
  while(centre == 1){
  dead = 3;
  delay(200);
//    printf(" inside while centre == 1 ");
    if(*x == 0 && *y == 0){
      *x = *x; // might not need this bit at all
      *y = *y;
    }
     scan(wall, xptr, yptr,walls);
     if(*x == junctions[n-1][0] && *y == junctions[n-1][1]){
      centre = 0; //sets the centre to 0
      dead = 0;   //sets dead to 0
   }
      wantedDirection = from_Centre(xptr,yptr, maze);;
      change_Direction();
      scan(wall, xptr, yptr, walls);
      move(wall, xptr, yptr);
  } 
}

//--------------------------------------------------------------------------------------------------------------------
// get_to_FUCK(maze,xptr, yptr)
int get_to_FUCK(int maze[9][9], int *x, int *y){

  int route = path + 1;   //defines route as path + 1

  //if the value in the cell is equal to route then set direction as stated
    if(maze[*x-1][*y] == route){
      wantedDirection = 0;   //set wantedDirection to 0
    }

  //if the value in the cell is equal to route then set direction as stated
    if(maze[*x][*y+1] == route){
      wantedDirection = 1;   //set wantedDirection to 1
    }

  //if the value in the cell is equal to route then set direction as stated
    if(maze[*x+1][*y] == route){
      wantedDirection = 2;   //set wantedDirection to 2
    }
  
  //if the value in the cell is equal to route then set direction as stated
    if(maze[*x][*y-1] == route){
      wantedDirection = 3;   //set wantedDirection to 3
    }
  return wantedDirection;
}
//---------------------------------------------------------------------------------------
//this function is used to determine if there is a shorter route to the centre
//better_Route(maze[9][9],xptr, yptr, map_Paths,wall)
int better_Route(int maze[9][9], int *x, int *y, int map_Paths[4], int current_Walls[4]){

  scan(wall, xptr, yptr,walls);
  for(int i=0; i<4; i++){
    map_Paths[i] = 0;     
  }
  
  int big_Path = path + 1;    //sets big path to path + 1 
  if(maze[*x-1][*y] == 98 || maze[*x-1][*y] > big_Path){
    map_Paths[0] = 1;   //sets map paths to 1
  }

  //if the cell in the stated direction is equal to 98 or larger than the current
  if(maze[*x][*y+1] == 98 || maze[*x][*y+1] > big_Path){
    map_Paths[1] = 1;   //sets map paths to 1
  }
  //if the cell in the stated direction is equal to 98 or larger than the current
  if(maze[*x+1][*y] == 98 || maze[*x+1][*y] > big_Path){
    map_Paths[2] = 1;   //sets map paths to 1
  }
  
  //if the cell in the stated direction is equal to 98 or larger than the current
  if(maze[*x][*y-1] == 98 || maze[*x][*y-1] > big_Path){
    map_Paths[3] = 1; }  //sets map paths to 1
  
  int gaps[4];
  for (int i = 0; i<4; i++){
    gaps[i]= (current_Walls[i] - 1) * (current_Walls[i] - 1);     //sets gaps to the opposite of current walls
  }
  
  int smaller_Paths = 0;
  smaller_Paths =(map_Paths[0] * gaps[0]) + (map_Paths[1] * gaps[1]) + (map_Paths[2] * gaps[2]) + (map_Paths[3] * gaps[3]);   //multiplies the map paths and gaps to give a total of smaller paths
  
  //if smaller paths is larger than 0
  if(smaller_Paths > 0){
    explored = 3;     //sets explored to 3
  }
}

//------------------------------------------------------------------------------------------------
//over_write(maze, xptr, yptr);
int over_Write(int maze[9][9], int *x, int *y){

    int route = path + 1;   //sets route to path + 1
  //if statement to determine if there is a cell filled with a path count larger than current path and below 99
    if(99 > maze[*x-1][*y] && maze[*x-1][*y] > path){
      wantedDirection = 0;     //sets wanted direction to 0
//      printf("\nwanted direction: %d\n", wantedDirection);
    }

  //if statement to determine if there is a cell filled with a path count larger than current path and below 99
    if(99 > maze[*x][*y+1] && maze[*x][*y+1] > path){
      wantedDirection = 1;     //sets wanted direction to 1
//      printf("\nwanted direction: %d\n", wantedDirection);
    }

  //if statement to determine if there is a cell filled with a path count larger than current path and below 99
    if(99 > maze[*x+1][*y] && maze[*x+1][*y] > path){
      wantedDirection = 2;     //sets wanted direction to 2
    }

  //if statement to determine if there is a cell filled with a path count larger than current path and below 99
    if(99 > maze[*x][*y-1] && maze[*x][*y-1] > path){
      wantedDirection = 3;     //sets wanted direction to 3
//      printf("\nwanted direction: %d\n", wantedDirection);
    }
  return wantedDirection;
}
//--------------------------------------------------------------------------------
//re-named from main as its a loop. 
//solve(xptr,yptr, wall)
int solve(int *x, int *y, int wall[4]){  
  //while statement for starting at coordinates (0, 0)
  while(xptr == 0 && yptr == 0 && count == 0){
    
    scan(wall, xptr, yptr, walls);
    maze_Update(xptr,yptr, maze);  //updates map with current path
    wantedDirection = find_Path(xptr, yptr, wall, maze);  //finds direction where the gap is
    change_Direction();   //changes direction
    move(wall, xptr, yptr);   //moves in correct direction
    maze_Update(xptr,yptr, maze); //updates map with current path count
  }
  count ++; //adds to count
  
  //while loop to map unexplored territory
  while(explored == 0){

    delay(200);
    found_Centre(xptr, yptr); //function to determine if current position is in the centre
    scan(wall, xptr, yptr, walls); //calls wall scan
    maze_Update(xptr,yptr, maze);   //updates map with current path

    if(explored == 0){

    wantedDirection = find_Path(xptr, yptr, wall, maze);  //finds direction where the gap is     //finds available paths and returns wanted direction
    dead_End_Check(xptr, yptr, wall, maze);

     better_Route(maze[9][9],xptr, yptr, map_Paths,wall);   //determines if there is a shorter route to the centre

    if(path >= shortest_Route){ //if the centre has been found and the shortest_Route is larger or equal to the current path count
      dead = 2;
      }


  //while statement to ensure the return to junction function is itterated through until there is no dead end
    while (dead == 1 || dead == 2){ 
   
        return_to_Junction(xptr, yptr, junctions);   //return to junction takes the mouse back to the previous junction
        if(explored ==! 0){   //if explored no longer equals 0 then the while loop is broken
        break;
    }
  }
    
   //if statement to define a junction
    if(wall[3] == 0 && wall[0] == 0 && wall[1] == 1 || wall[3] == 0 && wall[0] == 1 && wall[1] == 0 || wall[3] == 1 && wall[0] == 0 && wall[1] == 0 || wall[3] == 0 && wall[0] == 0 && wall[1] == 0){
        at_Junction(xptr,yptr,junctions);        //call junction
    }

    //changes direction to wanted direction
    change_Direction();

    //redifines walls after changing direction
    scan(wall, xptr, yptr, walls); 
    
  //checks can move forward and returns initialise move
    intialise_move = forward_Check(wall , xptr, yptr, intialise_move, maze);

    //if initialise move == 1
    if(intialise_move == 1){
      move(wall, xptr, yptr);
      scan(wall, xptr, yptr, walls);  //calls wall scan
      //calls map update
      maze_Update(xptr,yptr, maze);
    }
   }
    //calls stop
      stop(wall, xptr, yptr, maze);
}
  
  //while function which returns from centre to previous junction, checking for unmapped areas
  while(explored == 1){
      delay(200);
       scan(wall, xptr, yptr, walls);  //calls wall scan
       maze_Update(xptr,yptr, maze);
  
  //if statement that detects when the current position is at (0,0)
    if(*x == 0 && *y ==  0){
      explored = 2;   //changes explore to 2 so it exits this while function
      solve(xptr,yptr, wall);   //calls main function so that it will enter into explored == 2 while loop
    }

  //if centre is 1 return from centre, this value is changed later to ensure it doesnt get stuck
    if(centre == 1){  
      return_Centre(xptr,yptr);  //return from centre to previous junction
    }
    any_Unmapped(map_Paths, xptr, yptr, wall);  //checks the junctions for any unmapped area returning explore == 0 if there is
    if(explored == 0){
      solve(xptr,yptr, wall);   //calls main function to break this while loop
    }
    
    change_Direction(); //changes direction
    scan(wall, xptr, yptr, walls);  //rescans walls after direction has changed

    //checks can move forward and returns initialise move
    intialise_move = forward_Check(wall , xptr, yptr, intialise_move, maze);

  //if initialise move is '1' then move will be called
    if(intialise_move == 1){
      //calls move
      move(wall, xptr, yptr);
      //calls map update
      maze_Update(xptr,yptr, maze);
    }
  }

  //while function is used to return to the centre once the shortest route has been found and the mouse has returned to position (0,0)
   while(explored == 2){

    delay(200);  //delay
    scan(wall, xptr, yptr, walls);  //calls wall scan
    maze_Update(xptr,yptr, maze);   //updates map with current path

  //if statement to determine when the mouse has made it back to the centre
    if(*x == 4 && *y == 4){
      explored = 4;   //redifine explore so it does not enter any while loop in the main function
      done = 1;   //done to stop the function
      maze_Update(xptr,yptr, maze);
      solve(xptr,yptr, wall); //recall main with a different explore
    }
  
    get_to_FUCK(maze, xptr, yptr);  //function is used to return direction wanted along the shortest route which has been previously defined
    change_Direction();   //changes direction
    scan(wall, xptr, yptr, walls);    //scans the walls after a direction change
    intialise_move = forward_Check(wall , xptr, yptr, intialise_move, maze);  //checks to ensure there is a gap infront of the mouse

    if(intialise_move == 1){ //if there is a gap then the mouse can move
      move(wall, xptr, yptr);   //calls move
      maze_Update(xptr,yptr, maze); //calls map update
    }
   }

  //while loop that is used to overwrite a pathway with a shorter route
   while(explored == 3){
   delay(200);
   scan(wall, xptr, yptr, walls);  //calls wall scan
   maze_Update(xptr,yptr, maze);   //updates map with current path
  
  //if statement that detects a junction
    if(wall[3] == 0 && wall[0] == 0 && wall[1] == 1 || wall[3] == 0 && wall[0] == 1 && wall[1] == 0 || wall[3] == 1 && wall[0] == 0 && wall[1] == 0 || wall[3] == 0 && wall[0] == 0 && wall[1] == 0){
       at_Junction(xptr,yptr,junctions);    //call junction
      }
   
    //if at centre then the shortest_Route is redifined 
    if(*x == 4 && *y == 4){
      shortest_Route = path;  //set shortest_Route to current path count
      explored = 1;     //sets explored to '1' so put the programme back into while explored == 1
      solve(xptr,yptr, wall);     //calls main function
    }
  
    wantedDirection = over_Write(maze, xptr, yptr); //retrieves a wanted direction from the over_Write function
    change_Direction();   //changes direction
    scan(wall, xptr, yptr, walls);    //updates the walls after a change in direction
    intialise_move = forward_Check(wall , xptr, yptr, intialise_move, maze);    //forward check ensures there is a gap in front of the mouse

    if(intialise_move == 1){   //if there is a gap then the move function is called
      move(wall, xptr, yptr);   //calls move
      maze_Update(xptr,yptr, maze);   //calls map update
    }
   }
}