/*********************************************
 * OPL 22.1.1.0 Model
 * Author: Ayush
 * Creation Date: 15-Nov-2023 at 6:07:15 PM
 *********************************************/
tuple edge
{
   key int o;
   key int d;
   int weight;
}

{edge} edges=...;

{int} nodes={i.o | i in edges} union {i.d | i in edges};
int st=1; // start
int en=8; // end

//dvar int obj;  distance
dvar boolean x[edges]; // do we use that edge 

maximize sum(e in edges)
x[e]*e.weight;


subject to
{

// governing inflow and outflow of edges
forall(i in nodes: i != st && i != en){
    sum(e in edges: e.o == i) x[e] == sum(e in edges: e.d == i) x[e];
  }    
sum(e in edges: e.o == st) x[e] == 1;
sum(e in edges: e.d == en) x[e] == 1;
}
{edge} shortestPath={e | e in edges : x[e]==1};

execute
{
writeln(shortestPath);
}