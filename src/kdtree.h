/* \author Aaron Brown */
// Quiz on implementing kd tree

#include <vector>
#include <cmath>


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

class KdTree
{
public:
	Node* root;
	int dim;

	KdTree()
	: root(NULL)
	{}
	virtual ~KdTree(){
		delete_tree();
	}
	void delete_tree(){
		delete_tree_helper(root);
	}
	void delete_tree_helper(Node *&node){
		if(node == NULL) return;
		delete_tree_helper(node->left);
		delete_tree_helper(node->right);
		delete node;
		node = NULL;
	}


	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(root, 0, point, id);
	}

	void insertHelper(Node *&node, uint depth, std::vector<float> &point, int id){
		if(node == NULL){
			node = new  Node(point, id);
			return;
		}

		uint cd = depth % dim;
		if(point[cd] < node->point[cd]){
			insertHelper(node->left, depth+1, point, id);
		}else{
			insertHelper(node->right, depth+1, point, id);
		}

	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, distanceTol, 0, ids);
		return ids;
	}
	
	void searchHelper(std::vector<float> target, Node *node, float distanceTol, int depth, std::vector<int> &ids){
		if(node == NULL) return;

		//check if we need to add current node to the list
		bool bwithindistance = true;
		for(int d=0; d < dim; d++){
			if(node->point[d] < (target[d] - distanceTol) || node->point[d] > (target[d] + distanceTol)){
				bwithindistance = false;
				break;
			}
		}
		if(bwithindistance){
			float distance = 0;
			for(int d=0; d < dim; d++){
				distance += (node->point[d]-target[d]) * (node->point[d]-target[d]);
			}
			distance = sqrt(distance);
			if(distance <= distanceTol){
				ids.push_back(node->id);
			}
		}

		//check across boundary
		if ((target[depth%dim] - distanceTol) <= node->point[depth%dim] ){
			searchHelper(target, node->left, distanceTol, depth + 1, ids);
		}
		if ((target[depth%dim] + distanceTol) >= node->point[depth%dim]){
			searchHelper(target, node->right, distanceTol, depth + 1, ids);
		}
	}


};




