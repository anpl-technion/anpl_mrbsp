/* ---------------------------------------------------------------------------
 *
 * Autonomous Navigation and Perception Lab (ANPL),
 * Technion, Israel Institute of Technology,
 * Faculty of Aerospace Engineering,
 * Haifa, Israel, 32000
 * All Rights Reserved
 *
 * See LICENSE for the license information
 *
 * -------------------------------------------------------------------------- */
/**
 * @file: topology.h
 * @brief: Topological representation of a belief; Topological belief space planning
 * @author: Andrej Kitanov
 *
 */
#ifndef PLANNER_TOPOLOGY_H
#define PLANNER_TOPOLOGY_H

#include <Eigen/Dense>
#include <Eigen/Sparse>
//#include<Eigen/SparseLU>
#include<Eigen/SparseQR>
#include <Eigen/Eigenvalues>
#include <vector>

#include <planner/planner.h>
//#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <gvc.h>

using namespace cv;
using namespace Eigen;

typedef SparseMatrix<short> AdjacencyMatrix;
typedef SparseMatrix<int> LaplacianMatrix;
typedef SparseMatrix<double> NormLaplacianMatrix; // declares a column-major sparse matrix type of double
typedef Eigen::Matrix<unsigned int, Dynamic, 1> VectorXui;


class Topology;
enum signatures {VN_exact, VN, ST, VN_incr };
struct graph_elem_attrib {
    std::string color = "black";
    float width = 1;
};

class Graph {

    friend Topology;
public:
    /// graph described with Laplacian matrix and node degrees vector
    LaplacianMatrix L;
    VectorXi d;
    struct topological_metric {
        double s_VN_exact; // Von Neumann entropy of a graph
        double s_VN; // approx. of the Von Neumann entropy of a graph
        double s_VN_incr; // approx. of the Von Neumann entropy of a graph, calculated incrementally
        double s_ST; // signature based on the number of spanning trees
        double bounds_J[2]; // entropy bounds
        bool active[4]; // which of the above signatures are actually calculated and used in t-bsp
    } signature;

    MatrixXd Q; // edge noise model corresponding to pairwise factors pdf
    unsigned long n; // number of nodes in the current planning session
    unsigned long n_; // number of nodes in the previous planning session
    VectorXi d_; // node degrees in the previous planning session
    SparseVector < unsigned long > V_I; // involved vars
    AdjacencyMatrix A_; // connectivity of involved vars before applying control action
    std::vector< std::pair<unsigned long, unsigned long> > Delta_E; // new edges, vars indexes
    //std::vector< std::pair<unsigned long, unsigned long> > E_delta; // Markov blanket of involved vars without new edges
    unsigned int dimension; // node dimension, number of variables composing a node

    Graph() {
        // by default activate all signatures calculated from scratch
        signature.active[VN_exact] = false;
        signature.active[VN] = true;
        signature.active[ST] = true;
        signature.active[VN_incr] = false;
        signature.s_VN_incr = 0;

        n = 0;

    }

    unsigned long parseEdges(const string &edges, const std::string& edge_delim, const std::string& node_delim, bool addPath = true);
    std::vector< std::pair<char, unsigned long> > parseNodes(const string &nodes, const string& delim, bool replace = true, graph_elem_attrib* el_attr = NULL);
    std::string addEdgeAttribute(graph_elem_attrib& a);


    Graph(const AdjacencyMatrix& A);

    inline void setGraphFromAdjacency(const AdjacencyMatrix& other) { A = other; updateGraph(); };
    void updateGraph();
    void updateGraph(std::string &edges, std::string &nodes, bool replace);
    void calculateSignature();
    void updateGraphAndSignature(std::string& delta_edges, std::string& delta_nodes, gtsam::Values& new_values);
    double* TBSP_entropyBounds();

    // set all edges to have equal weights and information matrix
    void setEqualEdgeNoise(const MatrixXd& InfMat) {
        Q = InfMat.inverse();
        Omega = InfMat;
        Weights.setIdentity(n, n);
        dimension = Omega.rows();
    }

    /// return reference to the adjacency matrix of the graph
    AdjacencyMatrix& getAdjacencyMatrix() { return A; }
    /// set adjacency matrix of the graph
    void setAdjacencyMatrix(AdjacencyMatrix& AdjMat) { A = AdjMat; }
    /// show graph in Graphviz
    void showGraph();


    /// save graph data structures to a file
    bool saveGraph(const char* path, char* message = NULL);

private:
    AdjacencyMatrix A;
    //PermutationMatrix P; //P = perm_vector.asPermutation(); P'*M*P = M.twistedBy(P);
    VectorXi perm_vector;
    SparseMatrix<double> S; // scale matrix Ln = S*L*S
    // vector of row indices in adjacency matrix corresponding to some robot's states
    VectorXi robot_indices[NUM_ROBOTS];
    // temporary var, change in the number of states for each robot
    unsigned int dim_change[NUM_ROBOTS];
    // position of the last available empty slot in the vector robot_indices[r]
    //unsigned int pos[NUM_ROBOTS];
    gtsam::Values Embedding; // estimated vertex positions
    MatrixXd Omega; // edge information matrix
    MatrixXd Weights; // edge weights
    std::string dotgraph; // graph in dot language
};


class Topology {
public:
    Topology();

    /// create topology represented by a simple graph, assign it an embedding E and edge noise covariance Q
    void updatePrior(std::string& edges, std::string& nodes, gtsam::Values& E, MatrixXd& Omega, bool inBatchMode);

    /// create posterior topological graph
    void addSubgraph(std::string& delta_edges, std::string& delta_nodes);

    /// prior topological graph
    Graph prior_graph;

    /// posterior topological graphs
    std::vector<Graph> posterior_graphs;


    virtual ~Topology() {

        //cv::destroyWindow(OPENCV_WINDOW);
        time_ofs.close();
        ROS_WARN("Timing results saved.");

    }

    static std::string robotsNames;
    static std::ofstream time_ofs;
protected:
    std::string init_transf_edges;
    bool coordFramesConnected;

private:

    std::string OPENCV_WINDOW;
    void test_01();

    Graph tempGraphs[NUM_ROBOTS]; // topological graphs coressponding to local factor graphs
    std::string temp_mr_edges; // remaining edges

    GVC_t *gvc;

};

 // auxiliary functions for working with Eigen matrices
template <typename T, typename S> inline void sumAlongDim(
        const Eigen::SparseMatrix<T>& X,
        const int dim,
        Eigen::Matrix<S, Dynamic, 1>& v);

template<typename MatrixType>
void saveMatrixToFile(const char *filename, const MatrixType& m);

template<typename MatrixType>
void loadMatrixFromFile(const char *filename, MatrixType& m);

template<typename _Tp, int _rows, int _cols, int _options, int _maxRows, int _maxCols>
void eigen2cv(const Eigen::Matrix<_Tp, _rows, _cols, _options, _maxRows, _maxCols>& src, cv::Mat& dst);

#endif //PLANNER_TOPOLOGY_H
