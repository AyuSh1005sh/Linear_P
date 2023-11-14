/*********************************************
 * OPL 22.1.1.0 Model
 * Author: Ayush
 * Creation Date: 14-Nov-2023 at 4:11:32 AM
 *********************************************/

 dvar int x1;
 dvar int x2;
 
 maximize 5*x1+3*x2;
 subject to
 {
   6*x1 +6*x2<=24;
   x1+2*x2 <=6;
   x1+x2 <=1;
   
 }