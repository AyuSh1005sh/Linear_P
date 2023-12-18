using CP;
tuple edge {
  	key int id;
    key int o;
    key int d;
    int weight;
}

int n = ...; // number of drones
int m = ...; // number of packages
int k = ...; // number of edges
int tm = ...;// time scale

range I = 1..n;
range J = 1..m;
range E=1..k;
range T=1..tm;

int addr[J] = ...; // addr node of each package
{edge} Graph[I] = ...; // edge list of restricted path
{int} nodes[I]=...;
float Speed[I] = ...; // Speed of each drone
int AddressPresent[I][J] = ...; // Boolean grid indicating whether address is present in the graph or not
int PackageWeight[J]= ...;//weight of the package	
int DroneCap[I]= ...;// load capacity of the drone


// Define decision variables
// if x[i][j][t]=1 ,means ith drone will take the jth package at time 
// instant t
dvar boolean x[I][J][T];
// t[i][j] will hold the value for delivery time
dvar int t[I][J];
// gDecided[i][j][k]=1, means while delivering the jth package with ith 
// drone kth edge will be in the path
dvar boolean gDecided[I][J][E];

// Define objective function
dvar int obj;

minimize obj;

subject to {   
    // Governing inflow and outflow of nodes in each specified path
    forall(i in I, n in nodes[i],j in J, k in E: n != 1 && n != addr[j]){
    	sum(e in Graph[i]: e.o == n && e.id==k) gDecided[i][j][k]*AddressPresent[i][j] == sum(e in Graph[i]: e.d == n && e.id==k) gDecided[i][j][k]*AddressPresent[i][j];
       }    	
    forall(i in I, n in nodes[i],j in J, k in E){    
    	sum(e in Graph[i]: e.o == 1 && e.id==k) gDecided[i][j][k]*AddressPresent[i][j] == 1;
    	sum(e in Graph[i]: e.d == addr[j] && e.id==k) gDecided[i][j][k]*AddressPresent[i][j] == 1;    
       }    
    // Delivery Time constraints
    forall(i in I, j in J, k in E, tm in T) {
        t[i][j] == AddressPresent[i][j] * (sum(e in Graph[i]: e.id==k) (x[i][j][tm]*gDecided[i][j][k] * e.weight) / Speed[i]);
    }
    // Each package is assigned to exactly one drone
    forall(j in J) {
        sum(i in I, tm in T) x[i][j][tm] == 1;
    } 
    //weight constraint
    forall(i in I,j in J, tm in T){
      DroneCap[i]>=x[i][j][tm]*PackageWeight[j];
    }
    //time should not overlap
    forall(i in I,j in J,tm in T){     
      ((sum(tk in (tm+1..tm+t[i][j]) ) x[i][j][tk]==0) && x[i][j][tm]==1)==1;     
    } 
    // it should start the delivery from 0
    forall(i in I){
      sum(j in J)x[i][j][0] !=0;
    } 
    // Makespan for the I
    obj == max(i in I, j in J, tm in T)(tm*x[i][j][tm]+t[i][j]);
          
}
