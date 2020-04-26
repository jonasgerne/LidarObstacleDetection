#ifndef KDTREE_H
#define KDTREE_H
// Structure to represent node of kd tree
template<typename PointT>
struct Node {
    PointT point;
    int id;
    Node *left;
    Node *right;

    Node(PointT p, int setId)
            : point(p), id(setId), left(NULL), right(NULL) {}
};

template<typename PointT>
struct KdTree {
    Node<PointT> *root;

    KdTree() : root(NULL) {}

    void insertHelper(Node<PointT> **root_node, uint depth, const PointT point, int id) {
        if (*root_node == NULL) {
            *root_node = new Node<PointT>(point, id);
        } else {
            uint curr_dim = depth % 3;
            if (point.getVector3fMap()[curr_dim] > (*root_node)->point.getVector3fMap()[curr_dim])
                insertHelper(&((*root_node)->right), depth + 1, point, id); // get the double pointer of the right child
            else
                insertHelper(&((*root_node)->left), depth + 1, point, id);
        }
    }

    void insert(const PointT &point, int id) {
        // the function should create a new node and place correctly with in the root
        insertHelper(&root, 0, point, id);
    }

    void searchHelper(PointT target, Node<PointT> *root_node, int depth, float distanceTol, std::vector<int> &ids) {
        if (root_node != NULL) {
            // check if root is within box
            const auto& root_point = root_node->point.getVector3fMap();
            const auto& target_point_vec = target.getVector3fMap();
            if (root_point[0] <= (target_point_vec[0] + distanceTol) && root_point[0] >= (target_point_vec[0] - distanceTol)
                && root_point[1] <= (target_point_vec[1] + distanceTol) && root_point[1] >= (target_point_vec[1] - distanceTol)
                && root_point[2] <= (target_point_vec[2] + distanceTol) && root_point[2] >= (target_point_vec[2] - distanceTol)) {
                float distance = (target_point_vec - root_point).norm(); // sqrt(pow(target[0] - root_point[0], 2) + pow(target[1] - root_point[1], 2));
                if (distance <= distanceTol)
                    ids.emplace_back(root_node->id);
            }
            // check if left or right node is within box
            int dim = depth % 3;
            if (root_point[dim] > (target_point_vec[dim] - distanceTol))
                searchHelper(target, root_node->left, depth + 1, distanceTol, ids);
            if (root_point[dim] < (target_point_vec[dim] + distanceTol))
                searchHelper(target, root_node->right, depth + 1, distanceTol, ids);

        }
    }

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(const PointT &target, float distanceTol) {
        std::vector<int> ids;
        searchHelper(target, root, 0, distanceTol, ids);
        return ids;
    }
};
#endif




