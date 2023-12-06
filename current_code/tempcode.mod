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

range Drones = 1..n;
range Packages = 1..m;
range edges=1..k;
range Time=1..tm;

int addr[Packages] = ...; // addr node of each package
{edge} Graph[Drones] = ...; // edge list of restricted path
{int} nodes[Drones]=...;
float Speed[Drones] = ...; // Speed of each drone
int AddressPresent[Drones][Packages] = ...; // Boolean grid indicating whether address is present in the graph or not
int PackageWeight[Packages];
int DroneCap[Drones];


// Define decision variables
// if x[i][j][t]=1 ,t is the starting time
dvar boolean x[Drones][Packages][Time];
dvar int t[Drones][Packages];
dvar boolean gDecided[Drones][Packages][edges];

// Define objective function
dvar int obj;

minimize obj;

subject to {   
    // Governing inflow and outflow of nodes in each specified path
    forall(i in Drones, n in nodes[i],j in Packages, k in edges: n != 1 && n != addr[j]){
    	sum(e in Graph[i]: e.o == n && e.id==k) gDecided[i][j][k]*AddressPresent[i][j] == sum(e in Graph[i]: e.d == n && e.id==k) gDecided[i][j][k]*AddressPresent[i][j];
       }    	
    forall(i in Drones, n in nodes[i],j in Packages, k in edges){    
    	sum(e in Graph[i]: e.o == 1 && e.id==k) gDecided[i][j][k]*AddressPresent[i][j] == 1;
    	sum(e in Graph[i]: e.d == addr[j] && e.id==k) gDecided[i][j][k]*AddressPresent[i][j] == 1;    
       }    
    // Time constraints
    forall(i in Drones, j in Packages, k in edges, tm in Time) {
        t[i][j] >= AddressPresent[i][j] * (sum(e in Graph[i]: e.id==k) (x[i][j][tm]*gDecided[i][j][k] * e.weight) / Speed[i]);
    }
    // Each package is assigned to exactly one drone
    forall(j in Packages) {
        sum(i in Drones, tm in Time) x[i][j][tm] == 1;
    } 
    //weight constraint
    forall(i in Drones,j in Packages, tm in Time){
      DroneCap[i]>=x[i][j][tm]*PackageWeight[j];
    }
    //time should not overlap
    forall(i in Drones,j in Packages,tm in Time){     
      (sum(tk in (tm+1..tm+t[i][j]) )x[i][j][tk]==0) && x[i][j][tm]==1;     
    }  
    // Total time taken by drones
    obj == sum(i in Drones, j in Packages, tm in Time) (tm*x[i][j][tm]+t[i][j]);   
}
