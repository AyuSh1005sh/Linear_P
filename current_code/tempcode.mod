tuple edge {
  	key int id;
    key int o;
    key int d;
    int weight;
}

int n = ...; // number of drones
int m = ...; // number of packages
int k = ...; // number of edges

range Drones = 1..n;
range Packages = 1..m;
range edges=1..k;

int addr[Packages] = ...; // addr node of each package
{edge} Graph[Drones] = ...; // edge list of restricted path
{int} nodes[Drones]=...;
float Speed[Drones] = ...; // Speed of each drone
int AddressPresent[Drones][Packages] = ...; // Boolean grid indicating whether address is present in the graph or not

//execute{
//  for(var i in Drones){
//     nodes[i]= {e.o | e in Graph[i]} ;
//  }
//}

// Define decision variables
// if x[i][j][t]=1 ,t is the starting time
dvar boolean x[Drones][Packages];
dvar int t[Drones][Packages];
dvar boolean gDecided[Drones][edges];

// Define objective function
dvar float obj;

minimize obj;

subject to {   
    // Governing inflow and outflow of nodes in each specified path
    forall(i in Drones, n in nodes[i],j in Packages, k in edges: n != 1 && n != addr[j]){
    	sum(e in Graph[i]: e.o == n && e.id==k) gDecided[i][k]*AddressPresent[i][j] == sum(e in Graph[i]: e.d == n && e.id==k) gDecided[i][k]*AddressPresent[i][j];
       }    	
    forall(i in Drones, n in nodes[i],j in Packages, k in edges){    
    	sum(e in Graph[i]: e.o == 1 && e.id==k) gDecided[i][k]*AddressPresent[i][j] == 1;
    	sum(e in Graph[i]: e.d == addr[j] && e.id==k) gDecided[i][k]*AddressPresent[i][j] == 1;    
       }    
    // Time constraints
    forall(i in Drones, j in Packages, k in edges) {
        t[i][j] >= AddressPresent[i][j] * (sum(e in Graph[i]: e.id==k) (x[i][j]*gDecided[i][k] * e.weight) / Speed[i]);
    }
    // Each package is assigned to exactly one drone
    forall(j in Packages) {
        sum(i in Drones) x[i][j] == 1;
    }   
    // Total time taken by drones
    obj == sum(i in Drones, j in Packages) t[i][j];   
}
