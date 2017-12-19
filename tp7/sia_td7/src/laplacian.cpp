
#include "mesh.h"
#include "laplacian.h"
#include <Eigen/SparseCholesky>

using namespace Eigen;
using namespace surface_mesh;

typedef SparseMatrix<float> SpMat;
typedef PermutationMatrix<Dynamic> Permutation;
typedef Eigen::Triplet<double> T;

double cotan_weight(const Surface_mesh& mesh, Surface_mesh::Halfedge he)
{
    auto points = mesh.get_vertex_property<Point>("v:point");
    Surface_mesh::Vertex pj = mesh.to_vertex(he);
    Surface_mesh::Vertex pj1 = mesh.to_vertex(mesh.next_halfedge(he));

    he = mesh.opposite_halfedge(he);

    Surface_mesh::Vertex pi = mesh.to_vertex(he);
    Surface_mesh::Vertex pj2 = mesh.to_vertex(mesh.next_halfedge(he));

    Vector3f v2 = (points[pj2]-points[pi]);
    Vector3f v3 = (points[pj2]-points[pj]);
    Vector3f v4 = (points[pj1]-points[pi]);
    Vector3f v5 = (points[pj1]-points[pj]);

    double cosA = v2.dot(v3)/(v2.norm()*v3.norm());
    double sinA = v2.cross(v3).norm()/(v2.norm()*v3.norm());
    double cosB = v4.dot(v5)/(v4.norm()*v5.norm());
    double sinB = v4.cross(v5).norm()/(v4.norm()*v5.norm());

    return ((cosA/sinA)+(cosB/sinB))/2.0;
}

/// Computes the Laplacian matrix in matrix \a L using cotangent weights or the graph Laplacian if useCotWeights==false.
void create_laplacian_matrix(const Surface_mesh& mesh, SpMat& L, bool useCotWeights)
{
    // TODO

    // number of vertices in mesh
    int n = mesh.n_vertices();
    std::vector<T> tripletList;
    Mesh::Vertex_iterator vi = mesh.vertices_begin();
    double weightNeigbors = 0;
    for(;vi!=mesh.vertices_end();++vi)
    {
        Surface_mesh::Halfedge h = mesh.halfedge(*vi);
        Surface_mesh::Vertex first = mesh.to_vertex(h);
        Surface_mesh::Vertex vj = mesh.to_vertex(h);
        double weight = 0;
        do{
            if(useCotWeights){
                weight = cotan_weight(mesh,h);
            }else{
                weight = 1;
            }
            tripletList.push_back(T((*vi).idx(),vj.idx(),weight));
            h = mesh.next_halfedge(mesh.opposite_halfedge(h));
            vj = mesh.to_vertex(h);
            weightNeigbors += weight;
        }while(first!=vj);
        tripletList.push_back(T((*vi).idx(),(*vi).idx(),-weightNeigbors));
        weightNeigbors = 0;
    }
    L.setFromTriplets(tripletList.begin(), tripletList.end());

}

/// Computes the permutation putting selected vertices (mask==1) first, and the others at the end.
/// It returns the number of selected vertices.
int create_permutation(const Surface_mesh& mesh, Permutation & perm)
{
    auto masks = mesh.get_vertex_property<int>("v:mask");

    // number of vertices in mesh
    int n = mesh.n_vertices();

    // TODO
    int j=0;
    for(int i=0; i<n; ++i){
        if(masks.vector()[i]==1){
            perm.indices()[i] = j;
            j++;
        }
    }
    int res = j;
    for(int i = 0; i<n;++i){
        if(masks.vector()[i]==0){
            perm.indices()[i] = j;
            j++;
        }
    }
    return res;
}


/// Performs the poly-harmonic interpolation (order k) over the selected vertices (mask==1) of the vertex attributes u.
/// For each vertex V of index i,
///     if  mask[V]!=1 then u.col(i) is used as input constraints,
///     otherwise, mask[V}==1, and u.col(i) is replaced by the poly-harmonic interpolation of the fixed values.
void poly_harmonic_interpolation(const Surface_mesh& mesh, Ref<MatrixXf> u, int k)
{
    // Number of vertices in the mesh
    int n = mesh.n_vertices();

    // 1 - Create the sparse Laplacian matrix
    SpMat L(n,n);
//    create_laplacian_matrix(mesh, L, false);
    create_laplacian_matrix(mesh, L, true);
    SpMat tmpL=L;
    for(int i=0; i<k-1; i++){
        L = L*tmpL;
    }

    // 2 - Create the permutation matrix putting the fixed values at the end,
    //     and the true unknown at the beginning
    Permutation perm = Permutation(n);
    int nb_unknowns = create_permutation(mesh, perm);
    std::cout << nb_unknowns << std::endl;

    // 3 - Apply the permutation to both rows (equations) and columns (unknowns),
    //     i.e., L = P * L * P^-1

    // TODO
    L = L.twistedBy(perm);

    // 4 - solve L * [x^T u^T]^T = 0, i.e., L00 U = - L01 * C

    // TODO
    MatrixXf tmp = perm*(u.transpose());
    SpMat L00 = L.topLeftCorner(nb_unknowns,nb_unknowns);
    SpMat L01 = L.topRightCorner(nb_unknowns,n-nb_unknowns);
    Ref<MatrixXf> C = tmp.bottomRows(n-nb_unknowns);

    SimplicialLDLT<SparseMatrix<float>> solver;
    solver.compute(L00);
    if(solver.info()!=Success) {
        // decomposition failed
        std::cerr << "decomposition failed\n";
        return;
    }
    MatrixXf U = solver.solve(-L01*C);
    if(solver.info()!=Success) {
        // solving failed
        std::cerr << "solving failed\n";
        return;
    }

    // 5 - Copy back the results to u

    // TODO
    tmp.topRows(nb_unknowns) = U;
    u = MatrixXf(perm.inverse() * tmp).transpose();

}
