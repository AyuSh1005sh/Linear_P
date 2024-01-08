using CP;

tuple edge {
  	key int id;
    key int o;
    key int d;
    key int weight;
}

int n = ...; // number of drones
int m = ...; // number of packages
int k = ...; // number of edges
int t_max = ...;// time scale

range I = 1..n;
range J = 1..m;
range E = 1..k;
range T = 0..t_max;

int Dest[J] = ...; // addr node of each package
{edge} G[I] = ...; // edge list of restricted path
{int} V[I] = ...;
int Speed[I] = ...; // Speed of each drone
int AddressPresent[I][J] = ...; // Boolean grid indicating whether address is present in the graph or not
int W[J] = ...;//weight of the package	
int C[I] = ...;// load capacity of the drone


// Define decision variables
// if x[i][j][t]=1 ,means ith drone will take the jth package at time 
// instant t
dvar boolean x[I][J][T];
// t[i][j] will hold the value for delivery time
dvar int Tflight[I][J];

// Path[i][j][k]=1, means while delivering the jth package with ith 
// drone kth edge will be in the path
dvar boolean Path[I][J][E];

// Define objective function
dvar int obj;

minimize obj;

subject to {   
    // Governing inflow and outflow of nodes in each specified path
    C1:forall(i in I, j in J,n in V[i]: n != 1 && n != Dest[j]){
      if (AddressPresent[i][j] == 1) {
    	sum( e in G[i]:e.o == n ) Path[i][j][e.id] == sum(e in G[i]: e.d == n ) Path[i][j][e.id];
       }  }  
    C2: forall(i in I, j in J, e in E) {
    if (AddressPresent[i][j] == 1) {
        (sum(t in T) x[i][j][t] == 1) == (sum( e in G[i]: e.o == 1 ) Path[i][j][e.id] == 1);
    } else {
        (sum(t in T) x[i][j][t] == 0) == (sum( e in G[i]: e.o == 1) Path[i][j][e.id] == 0);
    }
}
    C3: forall(i in I, j in J, e in E) {
    if (AddressPresent[i][j] == 1) {
        (sum(t in T) x[i][j][t] == 1) == (sum( e in G[i]: e.d == Dest[j] ) Path[i][j][e.id] == 1);
    } else {
        (sum(t in T) x[i][j][t] == 0) == (sum( e in G[i]: e.d == Dest[j] ) Path[i][j][e.id] == 0);
    }
}
    // Delivery Time constraints
     C4: forall(i in I, j in J , t in T) {
       Tflight[i][j]>=0;
       if(AddressPresent[i][j]==1){
       Tflight[i][j] ==  sum(e in G[i] ) ((Path[i][j][e.id]*e.weight) / Speed[i]);
       } 
     }
     C4b: forall(i in I, j in J) {
      Tflight[i][j] == max(t in T) Tflight[i][j]*x[i][j][t];
     }
    // Each package is assigned to exactly one drone
    C5:forall(j in J) {
        sum(i in I, t in T) x[i][j][t] == 1;
    } 
    //weight constraint
    C6:forall(i in I,j in J, t in T){
      C[i]>=x[i][j][t]*W[j];
    }
   // time should not overlap
	C7: forall(i in I, j1 in J, j2 in J: j1 != j2, t1 in T, t2 in T:t1<=t2) {
    (t1 + Tflight[i][j1] <= t2) || (x[i][j1][t1] + x[i][j2][t2] <= 1);
     }
    // it should start the delivery from 0
     C8: sum(i in I,j in J)x[i][j][0] != 0;
    // Makespan for the I
    obj == max(i in I, j in J, t in T)(t*x[i][j][t]+Tflight[i][j]);
          
}