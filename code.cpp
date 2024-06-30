//The program computes the expected distances in a complete graph with edge weights in U[0,1]
//Note: During compilation the program requires C++ 9.4.0 and the following command for compilation:
// c++ file_name -lpthread (for compilation)
// ./a.out (for execution)


#include<bits/stdc++.h>
#include<pthread.h>
#include <thread>
#define k 20 //Number of times the distances are computed to take their average


using namespace std;
using namespace std::chrono;	//used for measuring time

class Graph {
public:
    int vertices;
    unordered_map<int, unordered_map<int, float>> graph;	//The adjcaceny list

    Graph(int v) : vertices(v) {}

    void addEdge(int u, int v, float weight) {
        graph[u][v] = weight;
        graph[v][u] = weight;
    }
	
	//The blackbox Djikstra's Algorithm which takes the source as input and return the distance to all other vertices as a vector
    vector<float> dijkstra(int start) {
        vector<bool> visited(vertices, false);
        vector<float> distance(vertices, DBL_MAX);
        distance[start] = 0;

        priority_queue<pair<float, int>, vector<pair<float, int>>, greater<pair<float, int>>> pq;
        pq.push({0, start});

        while (!pq.empty()) {
            int u = pq.top().second;
            pq.pop();

            if (visited[u]) {
                continue;
            }

            visited[u] = true;

            for (auto& neighbor : graph[u]) {
                int v = neighbor.first;
                float weight = neighbor.second;

                if (!visited[v]) {
                    float alt = distance[u] + weight;
                    if (alt < distance[v]) {
                        distance[v] = alt;
                        pq.push({alt, v});
                    }
                }
            }
        }

        return distance;
    }
};

//Computes the three distances for one instance
vector<float> Values(int n,int u, int v)
{

// Create a graph with random edge weights between 0 and 1
int i,j;
    	Graph graph(n);
    	for (int i = 0; i < n; ++i) {
    	    for (int j = i + 1; j < n; ++j) {
    	    	//Generates uniform random numbers b/w 0,1
    	        float weight = static_cast<float>(rand()) / RAND_MAX;
    	        
    	        //Omits edges with edge weights > 4 ln(n)/n 
    	        if(weight <= 4*log(n)/n)
    	        {
    	        graph.addEdge(i, j, weight);
    	        //cout<<i<<","<<j<<": "<<weight<<endl;
    	        }
    	    }
    	    //cout<<endl;
    	}
		

		
		float k1=0.0,k2=0.0,k3=0.0;
		//Vector for reading the distance vector
		vector<float> s_path;
		
		//Computes the X_{uv} and X_u
		s_path= graph.dijkstra(u);
		k1=s_path[v];
		for(j=1;j<n;j++)
		{
		k2=max(k2,s_path[j]);
		}	
		k3=k2;
		
		//Computes the diamter of the graph
		for(i=0;i<n;i++)
		{	
			if(i!=u)
			{
			s_path= graph.dijkstra(i);		
			for(j=i+1;j<n;j++)
			{
				k3=max(k3,s_path[j]);
    		}
    		s_path.clear();
    		}
    	}
    	vector<float> expected_values{k1,k2,k3};
    	
    	
    	//Sending the 3 values 
   		return expected_values;
}



//Computes and prints the expected distances over k instances
void print(int n, int u, int v)
{
		float mean_uv=0.0,mean_umaxv=0.0,mean_maxuv=0.0;

		int i;
		
		vector<thread> threads;
		
		vector<vector<float>> results;
		
		//Creates k threads and runs each instance of function Values() in each of the threads parallelly
		for (int i = 1; i <= k; ++i) {
        
        threads.push_back(thread([&results, n, u,v]() {
            vector<float> result = Values(n, u,v);
            results.push_back(result);
        }));}
			
		for (thread& t : threads) {t.join();}
    	//terminates the threads

		for(i=0;i<k;i++)
		{
		 //Extracting values from each thread
		mean_uv=mean_uv + results[i][0];
   		mean_umaxv=mean_umaxv + results[i][1];
   		mean_maxuv=mean_maxuv+results[i][2];
	
   		//collect each value from shared variable for each k
    	}//End of k iterations



   	//Computing the mean of each distance over k values
   	mean_uv=(float)mean_uv/k;
  	mean_umaxv=(float)mean_umaxv/k;
   	mean_maxuv=(float)mean_maxuv/k;
   		
   			
    //Printing the values of Expected fixeduv,	fixeduandmaxv,	maxoveralluv	
    cout<<setprecision(10);
    cout<<n<<","<<mean_uv<<","<<mean_umaxv<<","<<mean_maxuv<<",";
}





int main() {
   
   //The values of u and v can be varied
   int u=200,v=800;
    
    // Define the number of vertices in the graph
    for(int n=1000;n<=100000;n=n+1000)
	{
    auto t1 = high_resolution_clock::now();
	
	//To compute and print the expected values or each values of n
	print(n,u,v);
	
    auto t2 = high_resolution_clock::now();
 
    // Calculates total time taken for each value of n ( in milliseconds)
    auto ms_int = duration_cast<milliseconds>(t2 - t1);
	
	//Prints the time taken in minutes
    cout << (float)ms_int.count()/60000<<endl;
    }
    
    
    return 0;
}


