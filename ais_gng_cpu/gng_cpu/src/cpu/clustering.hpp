#include "../utils/utils.hpp"
#include "cugng.hpp"
#include "../utils/cluster.hpp"
#include "../clustering/dbscan.hpp"

#define CLUSTERING_STATIC_DISABLE -1

class Clustering {
    public:
     Clustering();
     ~Clustering();

     // Initialize clustering parameters
     void init(ClusterConfig *_cluster_config, CUGNG *_gng);

     void clustering();
    
    vector<Cluster> clusters;
    vector<Cluster> disable_clusters;
    int wall_cluster_size;
    int unknown_cluster_size;
    int safe_cluster_size;
    int human_cluster_size;

    ClusterConfig *cluster_config;
    CUGNG *gng;
    Time time;

    // DBSCAN dbscan;
    void _topologicalClustering(int idx, vector<int> &ids);
    void _topologicalClusteringExtention(int idx, vector<int> &ids, int label);
    void _topologicalClusteringOther(int idx, vector<int> &ids,
                                     bool static_node);
    void _take_over_cluster(Cluster &prev, Cluster &now);
    // void _topologicalClusteringUnknown(int idx, vector<int> &ids, bool static);
    // void _topologicalClusteringWall(int idx, vector<int> &ids);
    // void _topologicalCircleClustering(int idx, vector<int> &ids, Vec3f &center_pos);

    const uint32_t ykey1[4] = {_YK_KEY1_1, _YK_KEY1_2, _YK_KEY1_3, _YK_KEY1_4};
    const uint32_t fkey1[4] = {_FILE_KEY1_1, _FILE_KEY1_2, _FILE_KEY1_3, _FILE_KEY1_4};
};