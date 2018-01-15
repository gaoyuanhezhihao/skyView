#include "disjoint_set.h"
#include <iostream>
#include <exception>
#include <string>
#include <fstream>
#include <cassert>
using namespace std;
bool DisjointSet::SetUnion(int id1, int id2) {
    assert(0 <= id1 && id1 < set_roots.size());
    assert(0 <= id2 && id2 < set_roots.size());
    const int root_id1 = find(id1);
    const int root_id2 = find(id2);
    if(root_id1 == root_id2) {
        return false;
    }
    if(set_roots[root_id1] <= set_roots[root_id2]) {
        // set 1 is larger than set 2 or same size.
        set_roots[root_id1] = set_roots[root_id1] + set_roots[root_id2];
        set_roots[root_id2] = root_id1;
    } else {
        // set 1 is smaller than set 2.
        set_roots[root_id2] = set_roots[root_id1] + set_roots[root_id2];
        set_roots[root_id1] = root_id2;
    }
    return true;
}
bool DisjointSet::print() {
    size_t i = 0;
    cout << "id:    ";
    for(i = 0; i< set_roots.size(); ++i) {
        cout << i << "\t";
    }
    cout << endl;
    cout << "root: ";
    for(i = 0; i< set_roots.size(); ++i) {
        cout << set_roots[i]<< "\t";
    }
    cout << endl;
}

int DisjointSet::find(int dst_id) {
    if(dst_id >= int(set_roots.size()) ) {
        std::cout<< "!Error: DisjointSet::find: id is out of range." << endl;
        return -1;
    }
    if(set_roots[dst_id] <= -1) {
        return dst_id;
    }else {
        return set_roots[dst_id] = find(set_roots[dst_id]);
    }
}

bool DisjointSet::is_same_set(int l_id, int r_id) {
    const int l_root = find(l_id);
    const int r_root = find(r_id);
    if(l_root != -1 && r_root != -1 && l_root == r_root) {
        return true;
    } else {
        return false;
    }
}
bool DisjointSet::plot(const char name_plot[]) {
    const string name_graph(name_plot);
    const string file_name = name_graph + ".dot";
    ofstream file_stream(file_name);
    if(false == file_stream.is_open()) {
        return false;
    }
    file_stream << "digraph " << name_graph << "{\n";
    for(size_t i = 0; i< set_roots.size(); ++i) {
        if(set_roots[i] == -1) {
            // single node tree.
            file_stream << i << "[label=\"" << i << "\"]\n" ;
        }else if(set_roots[i] >= 0) {
            // it has a father.
            file_stream << set_roots[i] << " -> " << i << "\n";
        } else {
            // this is a root have some sons.
            file_stream << i << "[label=\"" << i << '|' << - set_roots[i] << "\"]\n";
        }
    }
    file_stream << "}" << endl;
    string cmd{"dot -Tpng "};
    cmd += file_name;
    cmd += (" -o ./"+ name_graph + ".png");
    std::cout << cmd << endl;
    system(cmd.c_str());
    return true;
}
