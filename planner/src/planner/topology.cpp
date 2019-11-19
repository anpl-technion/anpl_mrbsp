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
 * @file: topology.cpp
 * @brief: Topological representation of a belief; Topological belief space planning
 * @author: Andrej Kitanov
 *
 */
#include <planner/topology.h>
#include <bits/stdc++.h>

std::string Topology::robotsNames = "";
std::ofstream Topology::time_ofs;

Graph::Graph(const AdjacencyMatrix& other) {

    // by default activate all signatures
    signature.active[VN_exact] = false;
    signature.active[VN] = true;
    signature.active[ST] = true;
    signature.active[VN_incr] = true;

    setGraphFromAdjacency(other);

    PermutationMatrix<Dynamic> P;
    P.setIdentity(n); // no change of variable ordering
    perm_vector = P.indices();
}

// in batch mode replace the graph with the provided edges and nodes, in incremental add them to the graph
void Graph::updateGraph(std::string &edges, std::string &nodes, bool replace) {

    // by default activate all signatures and incremental VN only if it is not batch mode
    signature.active[VN_exact] = false;
    signature.active[VN] = true;
    signature.active[ST] = true;
    if (replace)
        signature.active[VN_incr] = false;
    else
        signature.active[VN_incr] = true;
    Delta_E.clear(); // clear the previous session

    char *robotsNames = &Topology::robotsNames[0u];


    // assuming connected graph => edges contain all the nodes in the graph, so we don't have to add them separately for drawing
    // here we only order and count nodes in the graph
    std::vector< std::pair<char, unsigned long> > new_nodes = parseNodes(nodes, ";", false, NULL);


    int i = 0;

    if (replace) { // batch
        n = new_nodes.size();
        n_ = n;
        V_I.resize(n_); //Resizes the sparse vector to newSize This method deletes all entries, thus leaving an empty sparse vector

        // resize and forget old variables
        for (int r = 0; r < NUM_ROBOTS; r++)
            robot_indices[r].resize(dim_change[r]);

        // set variable ordering
        for(std::vector< std::pair<char, unsigned long> >::iterator it = new_nodes.begin(); it != new_nodes.end(); it++, i++)
            robot_indices[it->first - 'A'][it->second] = i;


        // resize adjacency matrix
        A.resize(n,n);

    } else { // incremental

        //unsigned long n_ = n;
        A_ = A;
        n_ = n;
        n += new_nodes.size();
        ROS_WARN("G(k-1) size: %d", n_);
        V_I.resize(n_); //Resizes the sparse vector to newSize This method deletes all entries, thus leaving an empty sparse vector

        // make room for new variables
        for (int r = 0; r < NUM_ROBOTS; r++)
            robot_indices[r].conservativeResize(robot_indices[r].size() + dim_change[r]);

        // make variable ordering
        i = 0;
        for(std::vector< std::pair<char, unsigned long> >::iterator it = new_nodes.begin(); it != new_nodes.end(); it++, i++)
            robot_indices[it->first - 'A'][it->second] = n_ + i;

        // resize adjacency matrix
        A.conservativeResize(n,n);

    }


    // only for debugging
    // set permutation matrix such that first states of the robot A go, then B, ...
    // if we apply reordering P'*A*P

/*    perm_vector.resize(n);
    i = 0;
    for (int r = 0; r < NUM_ROBOTS; r++) {
        for (int s = 0; s < robot_indices[r].size(); s++, i++)
            perm_vector[i] = robot_indices[r][s];
    }
    PermutationMatrix<Dynamic> P(perm_vector.asPermutation());*/

    // up to this point adjacency matrix has been only resized to accommodate new states. This is where it gets updated with new edges
    // index of the node corresponding to the robot r and state i is equal to robot_indices[r - 'A'][i]
    parseEdges(edges, ";",  "--", replace);

    /*ROS_WARN("Involved vars = ");
    std::cout << V_I.toDense() << std::endl;*/
    printf("|\n|\n|\n");

    d_ = d; // node degrees at previous planning session

    updateGraph(); // update graph matrices

    // TODO this can be ignored, only for testing
    if (signature.active[VN_incr] && replace) { // in batch mode replace/reset all
        A_ = A;
        d_ = d;
        signature.s_VN_incr = 0;
    }



    //ROS_WARN_STREAM("After adding E, Dot graph is " << dotgraph);

    /*ROS_WARN("Adjacency matrix = ");
    std::cout << A << std::endl;

    ROS_WARN("Permutation matrix = ");
    std::cout << P.toDenseMatrix() << std::endl;*/

    //saveGraph(".", NULL);

}

void Graph::updateGraph() {

    n = A.rows();
    sumAlongDim(A, 1, d);
    SparseMatrix<int> D(n, n);
    VectorXd ds = d.cast<double>().cwiseSqrt().cwiseInverse();
    S.resize(n, n);
    for(int i = 0; i < n; i++) {
        D.coeffRef(i, i) = d(i);
        S.coeffRef(i, i) = ds(i);
    }
    L = D - A.cast<int>();
}

unsigned long Graph::parseEdges(const std::string& edges, const std::string& edge_delim, const std::string& node_delim, bool replace) {

    unsigned long n_E = 0;
    std::string dot_edges;
    graph_elem_attrib attrib;

    if (edges.size()) {
        typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
        boost::char_separator<char> sep(edge_delim.c_str());
        tokenizer tokens(edges, sep);
        for (tokenizer::iterator tok_iter = tokens.begin();
             tok_iter != tokens.end(); ++tok_iter, ++n_E) {
            //std::cout << "Found edge " << *tok_iter << " with nodes ";
            parseNodes(*tok_iter, node_delim, false, &attrib);
            dot_edges += *tok_iter + addEdgeAttribute(attrib) + "\n";
        }
        if (replace)
            dotgraph = dot_edges;
        else
            dotgraph += dot_edges;
    }

   return n_E;
}



std::vector< std::pair<char, unsigned long> > Graph::parseNodes(const std::string& nodes, const std::string& delim, bool addPath, graph_elem_attrib* el_attr) {

    unsigned long n_V = 0;
    memset(dim_change, 0, NUM_ROBOTS * sizeof(unsigned int));
    typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
    boost::char_separator<char> sep(delim.c_str());
    tokenizer tokens(nodes, sep);
    std::string odometry_path;
    char *prev_node;
    std::vector< std::pair<char, unsigned long> > nodes_out;
    std::pair<char, unsigned long> node;

    char *robotsNames = &Topology::robotsNames[0u];

    for (tokenizer::iterator tok_iter = tokens.begin();
         tok_iter != tokens.end(); ++tok_iter) {
        //std::cout << "Found node <" << *tok_iter << ">\n";
        std::size_t found = tok_iter->find_last_of(robotsNames, NUM_ROBOTS-1);
        if (found != std::string::npos) {
            char id = *(tok_iter->substr(0,found+1).c_str());
            unsigned long i = strtol(tok_iter->substr(found+1).c_str(), NULL, 0);
            //std::cout << " id = " << id << ", index = " << i << "\n";
            node = make_pair(id, i);
            nodes_out.push_back(node);
            ++n_V;
            dim_change[id - 'A']++;

            if (tok_iter == tokens.begin() && i > 0) {
                // connect path
                prev_node = new char(log(i)+2);
                sprintf(prev_node, "%c%ld--", id, i-1);
                odometry_path = prev_node + odometry_path;
                delete prev_node;
            }
            else
                odometry_path += *tok_iter + "--";
        }
    }
    if (addPath && odometry_path.length() > 2) {
        std::string linespec = "[color=red,penwidth=3.0];";
        odometry_path.replace(odometry_path.length()-2, 2, ""); // remove the last "--"
        dotgraph += odometry_path + linespec;
    }


    int i, j;
    std::pair<unsigned long, unsigned long> e_ij; // binary edge

    if (el_attr != NULL) {
        // add attributes to the graph element
        if (n_V == 2) { // binary edge

            i = robot_indices[nodes_out.at(0).first - 'A'][nodes_out.at(0).second];
            j = robot_indices[nodes_out.at(1).first - 'A'][nodes_out.at(1).second];

            if (i < n_)  {
                //ROS_WARN_STREAM(i << "is not a new node");
                V_I.coeffRef(i) = 1;
            }
            if (j < n_)  {
                //ROS_WARN_STREAM(j << "is not a new node");
                V_I.coeffRef(j) = 1;
            }

            e_ij.first = i;
            e_ij.second = j;
            if (!A.coeff(i, j)) // edge was added by the inference and planning => parallel edge -> collapse to a single edge, i.e. don't consider as a new edge
                Delta_E.push_back(e_ij);

            //for (int k = 0; k < mat.outerSize(); ++k)

           /* for (AdjacencyMatrix::InnerIterator it(A, i); it; ++it) {
                if (it.value()) ROS_WARN_STREAM(i << "is not a new node");
//                it.row();   // row index
//                it.col();   // col index (here it is equal to k, A is a column-major matrix)
//                it.index(); // inner index, here it is equal to it.row()
            }

            for (AdjacencyMatrix::InnerIterator it(A, j); it; ++it) {
                if (it.value()) ROS_WARN_STREAM(j << "is not a new node");
//                it.row();   // row index
//                it.col();   // col index (here it is equal to k, A is a column-major matrix)
//                it.index(); // inner index, here it is equal to it.row()
            }*/

            A.coeffRef(i, j) = 1;
            A.coeffRef(j, i) = 1;

            if (nodes_out.at(0).first == nodes_out.at(1).first) {
                if (abs(nodes_out.at(1).second - nodes_out.at(0).second) == 1) {// motion factor
                    el_attr->color = ROBOT_COLORS[nodes_out.at(0).first-'A'];
                    el_attr->width = 3.0;
                } else {
                    el_attr->color = "green";
                    el_attr->width = 3.0;
                }
            } else { // multi-robot factor
                el_attr->color = "blue";
                el_attr->width = 3.0;
            }
        } else if (n_V == 1) {
            // unary factor

        } else {
            // TODO hyperedge
        }
    }

    return nodes_out;
}

std::string Graph::addEdgeAttribute(graph_elem_attrib &a) {
    std::string s = "[color=" + a.color + ", penwidth = " + to_string(a.width) + "];";
    return s;
}


void Graph::showGraph() {

    // GraphViz
    GVC_t *gvc;
    Agraph_t *g;
    gvc = gvContext();
    std::string graphdot = "graph { rankdir=LR;\n" + dotgraph + " }";
    //ROS_WARN_STREAM("GRAPH DOT\n" << graphdot);
    g = agmemread(graphdot.c_str());
    gvLayout(gvc, g, "dot");
    //gvRender(gvc, g, "dot", stdout);
    gvRenderFilename(gvc, g, "dot", "out.dot");
    //gvRenderFilename(gvc, g, "svg", "out.svg");
    gvRenderFilename(gvc, g, "ps", "out.ps");
    //system("gv out.ps &");
    gvFreeLayout(gvc, g);
    agclose(g);
    gvFreeContext(gvc);
}

bool Graph::saveGraph(const char* path, char* message) {
    std::ofstream ofs;
    ofs.open (std::string(path) + "/A.txt", std::ofstream::out);
    if (message) ofs << message;
    ofs << A.toDense();
    ofs.close();

    ofs.open (std::string(path) + "/P.txt", std::ofstream::out);
    if (message) ofs << message;
    PermutationMatrix<Dynamic> P(perm_vector.asPermutation());
    ofs << P.toDenseMatrix();
    ofs.close();

    ofs.open (std::string(path) + "/robotStateDims.txt", std::ofstream::out);
    if (message) ofs << message;
    for (int i = 0; i < NUM_ROBOTS; i++)
        ofs << robot_indices[i].size() << " ";
    ofs.close();

}

void Graph::calculateSignature() {


    ROS_INFO("Signature, n = %d", n);

    if (signature.active[ST]) {
        gttic_(sigST);
        SparseMatrix<double> K(n-1, n-1); // reduced Laplacian matrix, Kirchoff matrix
        K = (L.block(1, 1, n - 1, n - 1)).cast<double>();
        SparseQR<SparseMatrix<double>, Eigen::COLAMDOrdering<int> > qr(K);
        qr.analyzePattern(K);
        qr.factorize(K);
        SparseMatrix<double> R = qr.matrixR();

        double tau = 0;
        for (unsigned i = 0; i < K.rows(); ++i)
            tau += std::log(fabs(R.coeff(i, i)));

        signature.s_ST = 1.5*tau + (n-1)/2.0*(log(Omega.determinant())-dimension*log(2*M_PI*exp(1)));
        gttoc_(sigST);
        tictoc_getNode(tsigST, sigST);
        Topology::time_ofs << tsigST.get()->wall() << " ";

        //ROS_INFO_STREAM("tau = " << tau);
        //ROS_INFO_STREAM("correction = " << (n-1)/2.0*(log(Omega.determinant())-dimension*log(2*M_PI*exp(1))) << ", det = " << Omega.determinant() << ", dim = " << dimension);
        ROS_INFO_STREAM("s_ST = " << signature.s_ST);
    }

    if (signature.active[VN_exact]) {
        gttic_(sigVNexact);
        NormLaplacianMatrix L_norm = S * L * S;
        //std::cout << "Normalized Laplacian: " << L_norm << std::endl;
        EigenSolver<MatrixXd> es(L_norm.toDense());
        //cout << "The eigenvalues of L_norm are:" << endl << es.eigenvalues() << endl;
        //cout << "The matrix of eigenvectors, V, is:" << endl << es.eigenvectors() << endl << endl;

        //saveMatrixToFile("L_n", L_norm.toDense());

        std::ptrdiff_t i0;
        es.eigenvalues().real().minCoeff(&i0); // i0 - index of the zero eigenvalue
        // Exact Von Neumann entropy of a graph
        signature.s_VN_exact = 0;
        for (unsigned i = 0; i < n; i++) {
            if (i == i0) continue;
            signature.s_VN_exact += es.eigenvalues()[i].real() / 2 * std::log(es.eigenvalues()[i].real() / 2);
            //cout << "lambda(" << i << ") = " << es.eigenvalues()[i].real() << endl;
        }
        signature.s_VN_exact *= -1;
        gttoc_(sigVNexact);
        ROS_INFO_STREAM("Von Neumann entropy: " << signature.s_VN_exact);
    }

    if (signature.active[VN]) {
        // Approximate Von Neumann entropy
        gttic_(sigVN);
        signature.s_VN = 0;
        for (int k = 0; k < A.outerSize(); ++k) {
            for (AdjacencyMatrix::InnerIterator it(A, k); it; ++it) {
                /*cout << it.value() << ", " <<
                     it.row() << ", " << // row index
                     it.col() << ", " <<   // col index (here it is equal to k)
                     it.index() << endl; // inner index, here it is equal to it.row()
                      */
                signature.s_VN += 1.0 / (d(it.row()) * d(it.col()));
            }
        }
        signature.s_VN = n * log(2) / 2 - signature.s_VN / 2;
        gttoc_(sigVN);
        tictoc_getNode(tsigVN, sigVN);
        Topology::time_ofs << tsigVN.get()->wall() << " ";
        ROS_INFO_STREAM("Von Neumann entropy approx: " << signature.s_VN);
    }

    if (signature.active[VN_incr]) {

        /******* signature s_VN calculated incrementally ****/

        // determine Markov Blanket of V_I without new edges = E_delta
        // this is on A_
        // d updated node degrees
        // d_ previous node degrees
        double delta_q = 0;
        AdjacencyMatrix MB_upper(n_,n_); //MB.triangularView<Upper>();
        MB_upper.reserve(V_I.size());
        MB_upper.setZero();

        gttic_(sigVNincr);
        //std::cout << "Markov blanket of involved vars in E(k-1)" << std::endl;
        for (SparseVector< unsigned long >::InnerIterator vit(V_I, 0); vit; ++vit ) { // outer dimension are columns/rows for ColMajor/RowMajor matrix
            for (AdjacencyMatrix::InnerIterator it(A_, vit.row()); it; ++it) {
                //cout << it.row() << "\t";
                //cout << it.col() << "\n";
                //cout << it.value() << endl;
                if (it.row() < it.col())
                    MB_upper.coeffRef(it.row(), it.col()) = it.value();
                else
                    MB_upper.coeffRef(it.col(), it.row()) = it.value();
            }
        }
        for (int k = 0; k < MB_upper.outerSize(); ++k) {
            for (AdjacencyMatrix::InnerIterator it(MB_upper, k); it; ++it) {
                //ROS_WARN_STREAM("(" << it.row() << ", " << it.col() << ")");
                delta_q += 1.0 / (d(it.row()) * d(it.col())) - 1.0 / (d_(it.row()) * d_(it.col()));
            }
        }

        // Delta_E contains new edges variable indexes
        //std::cout << "New edges" << std::endl;
        for(vector<std::pair<unsigned long, unsigned long>>::iterator it = Delta_E.begin(); it != Delta_E.end(); it++) {
            delta_q += 1.0/(d((*it).first) * d((*it).second));
            //cout << "Edge (" << (*it).first << ", " << (*it).second << ") with deg. " << d((*it).first) << " and " << d((*it).second) << endl;
        }
        signature.s_VN_incr += (n-n_)*log(2)/2 - delta_q; // we don't divide by 2 because (i,j) edge is counted in the sum but not (j,i)
        gttoc_(sigVNincr);
        tictoc_getNode(tsigVNincr, sigVNincr);
        Topology::time_ofs << tsigVNincr.get()->wall() << " ";
        ROS_INFO_STREAM("Von Neumann entropy approx incr: " << signature.s_VN_incr);

    }
}

void Graph::updateGraphAndSignature(std::string &delta_edges, std::string &delta_nodes, gtsam::Values &new_values) {

    ROS_WARN_STREAM("E = " << delta_edges << " |***| V = " << delta_nodes);

    gttic_(updateTBSPposterior);
    gttic_(updatePostGraph);
    updateGraph(delta_edges, delta_nodes, false);
    Embedding = new_values;
    setEqualEdgeNoise(Omega);
    gttoc_(updatePostGraph);
    tictoc_getNode(tupdatePostGraph, updatePostGraph);
    Topology::time_ofs << tupdatePostGraph.get()->wall() << " ";

    calculateSignature();

    gttoc_(updateTBSPposterior);

    showGraph();

    ROS_WARN("Topological graph and signatures updated.");
    tictoc_print_(); // Optional

}

double *Graph::TBSP_entropyBounds() {
    return nullptr;
}


Topology::Topology() :
        OPENCV_WINDOW("Sparsity pattern"),
        gvc (NULL),
        coordFramesConnected(false)
{


    Agraph_t *g;
    std::string dotgraph = "graph { graph [bb=\"0,0,640,480\"];\n"
            "\tnode [label=\"\\N\"];}";
    gvc = gvContext();
    g = agmemread(dotgraph.c_str());
    gvLayout(gvc, g, "dot");
    gvRenderFilename(gvc, g, "dot", "out.dot");
    //system("xdot -f neato out.dot &");
    //system("xdot -f dot out.dot &");
    //gvRenderFilename(gvc, g, "svg", "out.svg");
    //gvRenderFilename(gvc, g, "ps", "out.ps");
    //system("gv out.ps &");
    gvFreeLayout(gvc, g);
    agclose(g);
    gvFreeContext(gvc);

    robotsNames.resize(NUM_ROBOTS);
    for (int i = 0; i < NUM_ROBOTS; i++)
        robotsNames[i] =  ((char)'A' + i);
    std::cout << "Robots identifiers: " << robotsNames << std::endl;

    if (NUM_ROBOTS > 1) {
        for (int i = 0; i < NUM_ROBOTS - 1; i++) {
            std::stringstream e0;
            e0 << robotsNames[i] << "0--" << robotsNames[i + 1] << "0;";
            init_transf_edges += e0.str();
        }
    }

    Topology::time_ofs.open ("/home/andrej/timing.txt", std::ofstream::out);

    ROS_WARN("Topology init. OK");

    //namedWindow(OPENCV_WINDOW, WINDOW_AUTOSIZE );// Create a window for display.
    //namedWindow(OPENCV_WINDOW, CV_GUI_NORMAL );// Create a window for display.

    //test_01();

}


void spy(const Eigen::VectorXd& x, int n, std::string windowName)
{
    Eigen::Array<unsigned char,Eigen::Dynamic,Eigen::Dynamic> bits = (x*255).cast<unsigned char>();

    Mat image(n, n, CV_8UC1, bits.data());

    imshow(windowName, image );

    waitKey(10);

}

void Topology::test_01() {

    typedef Eigen::Triplet<double> T; // a non-zero entry as the triplet: row index, column index, value

    ROS_WARN("Running test_01");
    int n = 4;  // size of the image
    // Assembly:
    std::vector<T> coefficients;            // list of non-zeros coefficients
    coefficients.push_back(T(1,2,1));
    AdjacencyMatrix A(n,n);
    A.setFromTriplets(coefficients.begin(), coefficients.end());
    std::cout << "A = " << A << std::endl;
    MatrixXd Ad(A); // dense matrix A
    Map<RowVectorXd> v(Ad.data(), Ad.size());
    std::cout << "v = " << v << std::endl;


    // Show sparsity pattern of A
    spy(v, n, OPENCV_WINDOW);
    ROS_WARN_STREAM("Test passed ");


}

// create or update topological representation depending on the mode:
// 1) mode = 0 => incremental operation, edges and nodes are added to the graph
// 2) mode = 1 => batch operation, edges and nodes replace previous
void Topology::updatePrior(std::string &edges, std::string &nodes, gtsam::Values &E, MatrixXd &Omega, bool inBatchMode) {


    ROS_WARN("Updating prior topology ...");
    ROS_WARN_STREAM("Robots in a team are " << robotsNames[0] << robotsNames[1]);

    // add initial transforms (connections of coordinate frames) to the MR graph every time in batch mode,
    // and only once in incremental mode
    if ((!inBatchMode && !coordFramesConnected) || (inBatchMode) ) {
        edges += init_transf_edges;
        coordFramesConnected = true;
    }

    ROS_WARN_STREAM("E = " << edges << " |***| V = " << nodes);

    gttic_(updateTBSPprior);
    gttic_(updatePriorGraph);
    prior_graph.updateGraph(edges, nodes, inBatchMode);
    prior_graph.Embedding = E;
    prior_graph.setEqualEdgeNoise(Omega);
    gttoc_(updatePriorGraph);
    tictoc_getNode(tUpdatePriorGraph, updatePriorGraph);
    time_ofs << tUpdatePriorGraph.get()->wall() << " ";

    prior_graph.calculateSignature();
    gttoc_(updateTBSPprior);

    prior_graph.showGraph();

    ROS_WARN("Prior topology created in %s mode.", inBatchMode?"BATCH":"INCR");

}

void Topology::addSubgraph(std::string &delta_edges, std::string &delta_nodes) {

}

#include <planner/eigen_utils.hpp>
