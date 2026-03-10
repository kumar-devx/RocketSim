#pragma once

#include "shapes.cuh"

constexpr float MIN_AABB_DIMENSION = 0.002f;
constexpr float MIN_AABB_HALF_DIMENSION = MIN_AABB_DIMENSION / 2.0f;
constexpr float QUANTISATION_SCALE = 65533.0f;
constexpr float QUANTISATION_MARGIN = 1.0f;
constexpr int MAX_SUBTREE_SIZE_IN_BYTES = 2048;
constexpr int MAX_NODES_PER_SUBTREE = MAX_SUBTREE_SIZE_IN_BYTES / 16;

enum NodeType { NODE_LEAF, NODE_BRANCH };

RS_INLINE int max_variance_axis(const Vec3& v) {
    return v.x < v.y ? (v.y < v.z ? 2 : 1) : (v.x < v.z ? 2 : 0);
}

struct BvhNode {
    Aabb aabb;
    NodeType node_type;
    int triangle_index;
    int escape_index;

    RS_INLINE BvhNode() : node_type(NODE_LEAF), triangle_index(0), escape_index(0) {}
};

struct QuantisedBvhNode {
    unsigned short quantised_aabb_min[3];
    unsigned short quantised_aabb_max[3];
    int escape_index_or_triangle_index;

    RS_INLINE bool is_leaf_node() const {
        return escape_index_or_triangle_index >= 0;
    }

    RS_INLINE int get_escape_index() const {
        return -escape_index_or_triangle_index;
    }

    RS_INLINE int get_triangle_index() const {
        return escape_index_or_triangle_index;
    }
};

struct BvhSubtreeInfo {
    unsigned short quantised_aabb_min[3];
    unsigned short quantised_aabb_max[3];
    int root_node_index;
    int subtree_size;

    RS_INLINE void set_aabb_from_node(const QuantisedBvhNode& node) {
        quantised_aabb_min[0] = node.quantised_aabb_min[0];
        quantised_aabb_min[1] = node.quantised_aabb_min[1];
        quantised_aabb_min[2] = node.quantised_aabb_min[2];
        quantised_aabb_max[0] = node.quantised_aabb_max[0];
        quantised_aabb_max[1] = node.quantised_aabb_max[1];
        quantised_aabb_max[2] = node.quantised_aabb_max[2];
    }
};

struct QuantisedBvh {
    Vec3 bvh_aabb_min;
    Vec3 bvh_aabb_max;
    Vec3 bvh_quantisation;
    QuantisedBvhNode* nodes;
    QuantisedBvhNode* leaf_nodes;
    BvhSubtreeInfo* subtree_headers;
    int num_nodes;
    int cur_node_index;
    int num_subtree_headers;
    int subtree_header_capacity;

    RS_INLINE QuantisedBvh() : nodes(nullptr), leaf_nodes(nullptr), subtree_headers(nullptr),
        num_nodes(0), cur_node_index(0), num_subtree_headers(0), subtree_header_capacity(0) {}

    RS_INLINE void quantise(unsigned short* out, const Vec3& point, bool is_max) const {
        Vec3 v = (point - bvh_aabb_min) * bvh_quantisation;
        if (is_max) {
            out[0] = (unsigned short)(((unsigned short)(v.x + 1.0f)) | 1);
            out[1] = (unsigned short)(((unsigned short)(v.y + 1.0f)) | 1);
            out[2] = (unsigned short)(((unsigned short)(v.z + 1.0f)) | 1);
        } else {
            out[0] = (unsigned short)(((unsigned short)(v.x)) & 0xfffe);
            out[1] = (unsigned short)(((unsigned short)(v.y)) & 0xfffe);
            out[2] = (unsigned short)(((unsigned short)(v.z)) & 0xfffe);
        }
    }

    RS_INLINE void quantise_with_clamp(unsigned short* out, const Vec3& point, bool is_max) const {
        Vec3 clamped;
        clamped.x = fmaxf(bvh_aabb_min.x, fminf(bvh_aabb_max.x, point.x));
        clamped.y = fmaxf(bvh_aabb_min.y, fminf(bvh_aabb_max.y, point.y));
        clamped.z = fmaxf(bvh_aabb_min.z, fminf(bvh_aabb_max.z, point.z));
        quantise(out, clamped, is_max);
    }

    RS_INLINE Vec3 unquantise(const unsigned short* vec_in) const {
        return Vec3(
            (float)(vec_in[0]) / bvh_quantisation.x + bvh_aabb_min.x,
            (float)(vec_in[1]) / bvh_quantisation.y + bvh_aabb_min.y,
            (float)(vec_in[2]) / bvh_quantisation.z + bvh_aabb_min.z
        );
    }

    RS_INLINE unsigned int test_quantised_aabb_overlap(
        const unsigned short* aabb_min1, const unsigned short* aabb_max1,
        const unsigned short* aabb_min2, const unsigned short* aabb_max2) const {
        return (aabb_min1[0] <= aabb_max2[0]) & (aabb_max1[0] >= aabb_min2[0]) &
               (aabb_min1[2] <= aabb_max2[2]) & (aabb_max1[2] >= aabb_min2[2]) &
               (aabb_min1[1] <= aabb_max2[1]) & (aabb_max1[1] >= aabb_min2[1]);
    }

    void setup_quantisation(const Aabb& root_aabb) {
        bvh_aabb_min = root_aabb.min - Vec3(QUANTISATION_MARGIN, QUANTISATION_MARGIN, QUANTISATION_MARGIN);
        bvh_aabb_max = root_aabb.max + Vec3(QUANTISATION_MARGIN, QUANTISATION_MARGIN, QUANTISATION_MARGIN);

        Vec3 aabb_size = bvh_aabb_max - bvh_aabb_min;
        bvh_quantisation = Vec3(QUANTISATION_SCALE / aabb_size.x, QUANTISATION_SCALE / aabb_size.y, QUANTISATION_SCALE / aabb_size.z);

        unsigned short vec_in[3];
        quantise(vec_in, bvh_aabb_min, false);
        Vec3 v = unquantise(vec_in);
        bvh_aabb_min.x = fminf(bvh_aabb_min.x, v.x - QUANTISATION_MARGIN);
        bvh_aabb_min.y = fminf(bvh_aabb_min.y, v.y - QUANTISATION_MARGIN);
        bvh_aabb_min.z = fminf(bvh_aabb_min.z, v.z - QUANTISATION_MARGIN);

        aabb_size = bvh_aabb_max - bvh_aabb_min;
        bvh_quantisation = Vec3(QUANTISATION_SCALE / aabb_size.x, QUANTISATION_SCALE / aabb_size.y, QUANTISATION_SCALE / aabb_size.z);

        quantise(vec_in, bvh_aabb_max, true);
        v = unquantise(vec_in);
        bvh_aabb_max.x = fmaxf(bvh_aabb_max.x, v.x + QUANTISATION_MARGIN);
        bvh_aabb_max.y = fmaxf(bvh_aabb_max.y, v.y + QUANTISATION_MARGIN);
        bvh_aabb_max.z = fmaxf(bvh_aabb_max.z, v.z + QUANTISATION_MARGIN);

        aabb_size = bvh_aabb_max - bvh_aabb_min;
        bvh_quantisation = Vec3(QUANTISATION_SCALE / aabb_size.x, QUANTISATION_SCALE / aabb_size.y, QUANTISATION_SCALE / aabb_size.z);
    }

    Vec3 get_leaf_aabb_min(int index) const {
        return unquantise(leaf_nodes[index].quantised_aabb_min);
    }

    Vec3 get_leaf_aabb_max(int index) const {
        return unquantise(leaf_nodes[index].quantised_aabb_max);
    }

    int calc_splitting_axis(int start_index, int end_index) {
        int num_indices = end_index - start_index;
        Vec3 means = Vec3::zero();
        Vec3 variance = Vec3::zero();

        for (int i = start_index; i < end_index; i++) {
            Vec3 centre = (get_leaf_aabb_max(i) + get_leaf_aabb_min(i)) * 0.5f;
            means = means + centre;
        }
        means = means * (1.0f / (float)num_indices);

        for (int i = start_index; i < end_index; i++) {
            Vec3 centre = (get_leaf_aabb_max(i) + get_leaf_aabb_min(i)) * 0.5f;
            Vec3 diff = centre - means;
            Vec3 diff2(diff.x * diff.x, diff.y * diff.y, diff.z * diff.z);
            variance = variance + diff2;
        }
        variance = variance * (1.0f / ((float)num_indices - 1.0f));

        return max_variance_axis(variance);
    }

    void swap_leaf_nodes(int i, int split_index) {
        QuantisedBvhNode tmp = leaf_nodes[i];
        leaf_nodes[i] = leaf_nodes[split_index];
        leaf_nodes[split_index] = tmp;
    }

    int sort_and_calc_splitting_index(int start_index, int end_index, int split_axis) {
        int num_indices = end_index - start_index;
        Vec3 means = Vec3::zero();

        for (int i = start_index; i < end_index; i++) {
            Vec3 centre = (get_leaf_aabb_max(i) + get_leaf_aabb_min(i)) * 0.5f;
            means = means + centre;
        }
        means = means * (1.0f / (float)num_indices);

        float means_arr[3] = { means.x, means.y, means.z };
        float split_value = means_arr[split_axis];

        int split_index = start_index;
        for (int i = start_index; i < end_index; i++) {
            Vec3 centre = (get_leaf_aabb_max(i) + get_leaf_aabb_min(i)) * 0.5f;
            float centre_arr[3] = { centre.x, centre.y, centre.z };
            if (centre_arr[split_axis] > split_value) {
                swap_leaf_nodes(i, split_index);
                split_index++;
            }
        }

        int range_balanced_indices = num_indices / 3;
        bool unbalanced = ((split_index <= (start_index + range_balanced_indices)) ||
                          (split_index >= (end_index - 1 - range_balanced_indices)));
        if (unbalanced) {
            split_index = start_index + (num_indices >> 1);
        }

        return split_index;
    }

    void set_internal_node_aabb_min(int node_index, const Vec3& aabb_min) {
        quantise(nodes[node_index].quantised_aabb_min, aabb_min, false);
    }

    void set_internal_node_aabb_max(int node_index, const Vec3& aabb_max) {
        quantise(nodes[node_index].quantised_aabb_max, aabb_max, true);
    }

    void merge_internal_node_aabb(int node_index, const Vec3& new_aabb_min, const Vec3& new_aabb_max) {
        unsigned short quantised_min[3];
        unsigned short quantised_max[3];
        quantise(quantised_min, new_aabb_min, false);
        quantise(quantised_max, new_aabb_max, true);
        for (int i = 0; i < 3; i++) {
            if (nodes[node_index].quantised_aabb_min[i] > quantised_min[i])
                nodes[node_index].quantised_aabb_min[i] = quantised_min[i];
            if (nodes[node_index].quantised_aabb_max[i] < quantised_max[i])
                nodes[node_index].quantised_aabb_max[i] = quantised_max[i];
        }
    }

    void set_internal_node_escape_index(int node_index, int escape_index) {
        nodes[node_index].escape_index_or_triangle_index = -escape_index;
    }

    void assign_internal_node_from_leaf_node(int internal_node, int leaf_node_index) {
        nodes[internal_node] = leaf_nodes[leaf_node_index];
    }

    void add_subtree_header(const BvhSubtreeInfo& header) {
        if (num_subtree_headers >= subtree_header_capacity) {
            int new_capacity = subtree_header_capacity == 0 ? 16 : subtree_header_capacity * 2;
            BvhSubtreeInfo* new_headers = new BvhSubtreeInfo[new_capacity];
            for (int i = 0; i < num_subtree_headers; i++) {
                new_headers[i] = subtree_headers[i];
            }
            if (subtree_headers) delete[] subtree_headers;
            subtree_headers = new_headers;
            subtree_header_capacity = new_capacity;
        }
        subtree_headers[num_subtree_headers++] = header;
    }

    void update_subtree_headers(int left_child_index, int right_child_index) {
        const QuantisedBvhNode& left_child = nodes[left_child_index];
        int left_subtree_size = left_child.is_leaf_node() ? 1 : left_child.get_escape_index();
        int left_subtree_bytes = left_subtree_size * (int)sizeof(QuantisedBvhNode);

        const QuantisedBvhNode& right_child = nodes[right_child_index];
        int right_subtree_size = right_child.is_leaf_node() ? 1 : right_child.get_escape_index();
        int right_subtree_bytes = right_subtree_size * (int)sizeof(QuantisedBvhNode);

        if (left_subtree_bytes <= MAX_SUBTREE_SIZE_IN_BYTES) {
            BvhSubtreeInfo subtree;
            subtree.set_aabb_from_node(left_child);
            subtree.root_node_index = left_child_index;
            subtree.subtree_size = left_subtree_size;
            add_subtree_header(subtree);
        }

        if (right_subtree_bytes <= MAX_SUBTREE_SIZE_IN_BYTES) {
            BvhSubtreeInfo subtree;
            subtree.set_aabb_from_node(right_child);
            subtree.root_node_index = right_child_index;
            subtree.subtree_size = right_subtree_size;
            add_subtree_header(subtree);
        }
    }

    void build_tree_recursive(int start_index, int end_index) {
        int num_indices = end_index - start_index;
        int cur_index = cur_node_index;

        if (num_indices == 1) {
            assign_internal_node_from_leaf_node(cur_node_index, start_index);
            cur_node_index++;
            return;
        }

        int split_axis = calc_splitting_axis(start_index, end_index);
        int split_index = sort_and_calc_splitting_index(start_index, end_index, split_axis);

        int internal_node_index = cur_node_index;

        set_internal_node_aabb_min(cur_node_index, bvh_aabb_max);
        set_internal_node_aabb_max(cur_node_index, bvh_aabb_min);

        for (int i = start_index; i < end_index; i++) {
            merge_internal_node_aabb(cur_node_index, get_leaf_aabb_min(i), get_leaf_aabb_max(i));
        }

        cur_node_index++;

        int left_child_index = cur_node_index;
        build_tree_recursive(start_index, split_index);

        int right_child_index = cur_node_index;
        build_tree_recursive(split_index, end_index);

        int escape_index = cur_node_index - cur_index;

        int tree_size_bytes = escape_index * (int)sizeof(QuantisedBvhNode);
        if (tree_size_bytes > MAX_SUBTREE_SIZE_IN_BYTES) {
            update_subtree_headers(left_child_index, right_child_index);
        }

        set_internal_node_escape_index(internal_node_index, escape_index);
    }

    void build(BvhNode* bvh_leaf_nodes, int num_leaf_nodes, Aabb root_aabb) {
        setup_quantisation(root_aabb);

        cur_node_index = 0;
        num_nodes = 2 * num_leaf_nodes;
        nodes = new QuantisedBvhNode[num_nodes];
        leaf_nodes = new QuantisedBvhNode[num_leaf_nodes];

        if (subtree_headers) {
            delete[] subtree_headers;
            subtree_headers = nullptr;
        }
        num_subtree_headers = 0;
        subtree_header_capacity = 0;

        for (int i = 0; i < num_leaf_nodes; i++) {
            quantise(leaf_nodes[i].quantised_aabb_min, bvh_leaf_nodes[i].aabb.min, false);
            quantise(leaf_nodes[i].quantised_aabb_max, bvh_leaf_nodes[i].aabb.max, true);
            leaf_nodes[i].escape_index_or_triangle_index = bvh_leaf_nodes[i].triangle_index;
        }

        build_tree_recursive(0, num_leaf_nodes);

        num_nodes = cur_node_index;

        int total_tree_bytes = num_nodes * (int)sizeof(QuantisedBvhNode);
        if (total_tree_bytes <= MAX_SUBTREE_SIZE_IN_BYTES && num_nodes > 0) {
            BvhSubtreeInfo root_subtree;
            root_subtree.set_aabb_from_node(nodes[0]);
            root_subtree.root_node_index = 0;
            root_subtree.subtree_size = num_nodes;
            add_subtree_header(root_subtree);
        }

        delete[] leaf_nodes;
        leaf_nodes = nullptr;
    }

    void build_subtree_headers_from_nodes_recursive(int node_index) {
        if (node_index >= num_nodes) return;

        const QuantisedBvhNode& node = nodes[node_index];
        if (node.is_leaf_node()) return;

        int escape_index = node.get_escape_index();
        int tree_size_bytes = escape_index * (int)sizeof(QuantisedBvhNode);

        if (tree_size_bytes > MAX_SUBTREE_SIZE_IN_BYTES) {
            int left_child_index = node_index + 1;
            const QuantisedBvhNode& left_child = nodes[left_child_index];
            int left_subtree_size = left_child.is_leaf_node() ? 1 : left_child.get_escape_index();
            int left_subtree_bytes = left_subtree_size * (int)sizeof(QuantisedBvhNode);

            int right_child_index = left_child_index + left_subtree_size;
            const QuantisedBvhNode& right_child = nodes[right_child_index];
            int right_subtree_size = right_child.is_leaf_node() ? 1 : right_child.get_escape_index();
            int right_subtree_bytes = right_subtree_size * (int)sizeof(QuantisedBvhNode);

            if (left_subtree_bytes <= MAX_SUBTREE_SIZE_IN_BYTES) {
                BvhSubtreeInfo subtree;
                subtree.set_aabb_from_node(left_child);
                subtree.root_node_index = left_child_index;
                subtree.subtree_size = left_subtree_size;
                add_subtree_header(subtree);
            } else {
                build_subtree_headers_from_nodes_recursive(left_child_index);
            }

            if (right_subtree_bytes <= MAX_SUBTREE_SIZE_IN_BYTES) {
                BvhSubtreeInfo subtree;
                subtree.set_aabb_from_node(right_child);
                subtree.root_node_index = right_child_index;
                subtree.subtree_size = right_subtree_size;
                add_subtree_header(subtree);
            } else {
                build_subtree_headers_from_nodes_recursive(right_child_index);
            }
        }
    }

    void build_subtree_headers_from_nodes() {
        if (subtree_headers) {
            delete[] subtree_headers;
            subtree_headers = nullptr;
        }
        num_subtree_headers = 0;
        subtree_header_capacity = 0;

        if (num_nodes == 0) return;

        int total_tree_bytes = num_nodes * (int)sizeof(QuantisedBvhNode);
        if (total_tree_bytes <= MAX_SUBTREE_SIZE_IN_BYTES) {
            BvhSubtreeInfo root_subtree;
            root_subtree.set_aabb_from_node(nodes[0]);
            root_subtree.root_node_index = 0;
            root_subtree.subtree_size = num_nodes;
            add_subtree_header(root_subtree);
        } else {
            build_subtree_headers_from_nodes_recursive(0);
        }
    }

    bool load_from_binary(const unsigned char* data, size_t size) {
        if (size < 44) return false;

        const float* aabb_min_ptr = reinterpret_cast<const float*>(data);
        const float* aabb_max_ptr = reinterpret_cast<const float*>(data + 12);
        const float* quant_ptr = reinterpret_cast<const float*>(data + 24);
        const int* node_count_ptr = reinterpret_cast<const int*>(data + 36);

        bvh_aabb_min = Vec3(aabb_min_ptr[0], aabb_min_ptr[1], aabb_min_ptr[2]);
        bvh_aabb_max = Vec3(aabb_max_ptr[0], aabb_max_ptr[1], aabb_max_ptr[2]);
        bvh_quantisation = Vec3(quant_ptr[0], quant_ptr[1], quant_ptr[2]);
        num_nodes = *node_count_ptr;

        size_t expected_size = 44 + num_nodes * 16;
        if (size < expected_size) return false;

        if (nodes) delete[] nodes;
        nodes = new QuantisedBvhNode[num_nodes];

        const unsigned char* node_data = data + 44;
        for (int i = 0; i < num_nodes; i++) {
            const unsigned short* mins = reinterpret_cast<const unsigned short*>(node_data);
            const unsigned short* maxs = reinterpret_cast<const unsigned short*>(node_data + 6);
            const int* escape_or_tri = reinterpret_cast<const int*>(node_data + 12);

            nodes[i].quantised_aabb_min[0] = mins[0];
            nodes[i].quantised_aabb_min[1] = mins[1];
            nodes[i].quantised_aabb_min[2] = mins[2];
            nodes[i].quantised_aabb_max[0] = maxs[0];
            nodes[i].quantised_aabb_max[1] = maxs[1];
            nodes[i].quantised_aabb_max[2] = maxs[2];
            nodes[i].escape_index_or_triangle_index = *escape_or_tri;

            node_data += 16;
        }

        cur_node_index = num_nodes;
        build_subtree_headers_from_nodes();
        return true;
    }

    template<typename Callback>
    RS_INLINE void walk_stackless_quantised_tree(
        Callback& cb,
        const unsigned short* query_min, const unsigned short* query_max,
        int start_node_index, int end_node_index) const {
        int cur_index = start_node_index;

        while (cur_index < end_node_index) {
            const QuantisedBvhNode& node = nodes[cur_index];

            bool overlap = test_quantised_aabb_overlap(
                query_min, query_max,
                node.quantised_aabb_min, node.quantised_aabb_max
            );

            bool is_leaf = node.is_leaf_node();

            if (is_leaf && overlap) {
                cb.process_node(node.get_triangle_index());
            }

            if (overlap || is_leaf) {
                cur_index++;
            } else {
                cur_index += node.get_escape_index();
            }
        }
    }

    template<typename Callback>
    RS_INLINE void report_aabb_overlapping_node(Callback& cb, const Aabb& test) const {
        unsigned short quantised_query_min[3];
        unsigned short quantised_query_max[3];
        quantise_with_clamp(quantised_query_min, test.min, false);
        quantise_with_clamp(quantised_query_max, test.max, true);

        if (num_subtree_headers > 0) {
            for (int i = 0; i < num_subtree_headers; i++) {
                const BvhSubtreeInfo& subtree = subtree_headers[i];

                bool overlap = test_quantised_aabb_overlap(
                    quantised_query_min, quantised_query_max,
                    subtree.quantised_aabb_min, subtree.quantised_aabb_max
                );

                if (overlap) {
                    walk_stackless_quantised_tree(
                        cb, quantised_query_min, quantised_query_max,
                        subtree.root_node_index,
                        subtree.root_node_index + subtree.subtree_size
                    );
                }
            }
        } else {
            walk_stackless_quantised_tree(
                cb, quantised_query_min, quantised_query_max,
                0, num_nodes
            );
        }
    }

    RS_INLINE bool ray_aabb_test(
        const Vec3& ray_from,
        const Vec3& ray_inv_direction,
        const unsigned int* sign,
        const Vec3* bounds,
        float& tmin,
        float lambda_min,
        float lambda_max) const {
        float tmax, tymin, tymax, tzmin, tzmax;

        tmin = (bounds[sign[0]].x - ray_from.x) * ray_inv_direction.x;
        tmax = (bounds[1 - sign[0]].x - ray_from.x) * ray_inv_direction.x;

        tymin = (bounds[sign[1]].y - ray_from.y) * ray_inv_direction.y;
        tymax = (bounds[1 - sign[1]].y - ray_from.y) * ray_inv_direction.y;

        if ((tmin > tymax) || (tymin > tmax))
            return false;

        if (tymin > tmin) tmin = tymin;
        if (tymax < tmax) tmax = tymax;

        tzmin = (bounds[sign[2]].z - ray_from.z) * ray_inv_direction.z;
        tzmax = (bounds[1 - sign[2]].z - ray_from.z) * ray_inv_direction.z;

        if ((tmin > tzmax) || (tzmin > tmax))
            return false;

        if (tzmin > tmin) tmin = tzmin;
        if (tzmax < tmax) tmax = tzmax;

        return (tmin < lambda_max) && (tmax > lambda_min);
    }

    RS_INLINE Vec3 min_vec(const Vec3& a, const Vec3& b) const {
        return Vec3(fminf(a.x, b.x), fminf(a.y, b.y), fminf(a.z, b.z));
    }

    RS_INLINE Vec3 max_vec(const Vec3& a, const Vec3& b) const {
        return Vec3(fmaxf(a.x, b.x), fmaxf(a.y, b.y), fmaxf(a.z, b.z));
    }

    template<typename Callback>
    RS_INLINE void report_box_cast_overlapping_node(
        Callback& cb,
        const Vec3& ray_source, const Vec3& ray_target,
        const Vec3& aabb_min, const Vec3& aabb_max) const {
        if (num_nodes == 0) return;

        Vec3 ray_dir = ray_target - ray_source;
        float ray_len_sq = ray_dir.length_squared();
        if (ray_len_sq < SIMD_EPSILON * SIMD_EPSILON) return;
        float inv_ray_len = rsqrtf(ray_len_sq);
        ray_dir = ray_dir * inv_ray_len;
        float lambda_max = ray_len_sq * inv_ray_len;

        Vec3 ray_inv_dir;
        ray_inv_dir.x = (fabsf(ray_dir.x) < SIMD_EPSILON) ? LARGE_FLOAT : 1.0f / ray_dir.x;
        ray_inv_dir.y = (fabsf(ray_dir.y) < SIMD_EPSILON) ? LARGE_FLOAT : 1.0f / ray_dir.y;
        ray_inv_dir.z = (fabsf(ray_dir.z) < SIMD_EPSILON) ? LARGE_FLOAT : 1.0f / ray_dir.z;

        unsigned int sign[3] = {
            ray_inv_dir.x < 0.0f ? 1u : 0u,
            ray_inv_dir.y < 0.0f ? 1u : 0u,
            ray_inv_dir.z < 0.0f ? 1u : 0u
        };

        Vec3 ray_aabb_min = min_vec(ray_source, ray_target) + aabb_min;
        Vec3 ray_aabb_max = max_vec(ray_source, ray_target) + aabb_max;

        unsigned short query_min[3], query_max[3];
        quantise_with_clamp(query_min, ray_aabb_min, false);
        quantise_with_clamp(query_max, ray_aabb_max, true);

        int cur_index = 0;
        while (cur_index < num_nodes) {
            const QuantisedBvhNode& node = nodes[cur_index];

            bool box_overlap = test_quantised_aabb_overlap(
                query_min, query_max,
                node.quantised_aabb_min, node.quantised_aabb_max
            );

            bool ray_overlap = false;
            if (box_overlap) {
                Vec3 bounds[2];
                bounds[0] = unquantise(node.quantised_aabb_min) - aabb_max;
                bounds[1] = unquantise(node.quantised_aabb_max) - aabb_min;

                float param = 1.0f;
                ray_overlap = ray_aabb_test(ray_source, ray_inv_dir, sign, bounds, param, 0.0f, lambda_max);
            }

            bool is_leaf = node.is_leaf_node();

            if (is_leaf && ray_overlap) {
                cb.process_node(node.get_triangle_index());
            }

            if (ray_overlap || is_leaf) {
                cur_index++;
            } else {
                cur_index += node.get_escape_index();
            }
        }
    }

    template<typename Callback>
    RS_INLINE void walk_stackless_quantised_tree_ray(
        Callback& cb,
        const Vec3& ray_from,
        const Vec3& ray_inv_dir,
        const unsigned int* sign,
        const unsigned short* query_min, const unsigned short* query_max,
        float lambda_max,
        float& t_max,
        int start_node_index, int end_node_index) const {
        int cur_index = start_node_index;

        while (cur_index < end_node_index) {
            const QuantisedBvhNode& node = nodes[cur_index];

            bool box_overlap = test_quantised_aabb_overlap(
                query_min, query_max,
                node.quantised_aabb_min, node.quantised_aabb_max
            );

            bool ray_overlap = false;
            if (box_overlap) {
                Vec3 bounds[2];
                bounds[0] = unquantise(node.quantised_aabb_min);
                bounds[1] = unquantise(node.quantised_aabb_max);

                float param = 1.0f;
                ray_overlap = ray_aabb_test(ray_from, ray_inv_dir, sign, bounds, param, 0.0f, lambda_max);
            }

            bool is_leaf = node.is_leaf_node();

            if (is_leaf && ray_overlap) {
                cb.process_ray_node(node.get_triangle_index(), t_max);
            }

            if (ray_overlap || is_leaf) {
                cur_index++;
            } else {
                cur_index += node.get_escape_index();
            }
        }
    }

    template<typename Callback>
    RS_INLINE void report_ray_overlapping_node(Callback& cb, const Vec3& origin, const Vec3& direction, float& t_max) const {
        if (t_max <= 0) return;
        if (num_nodes == 0) return;

        Vec3 target = origin + direction * t_max;

        Vec3 ray_dir = direction;
        float ray_len_sq = ray_dir.length_squared();
        if (ray_len_sq < SIMD_EPSILON * SIMD_EPSILON) return;
        float inv_ray_len = rsqrtf(ray_len_sq);
        ray_dir = ray_dir * inv_ray_len;
        float lambda_max = ray_len_sq * inv_ray_len * t_max;

        Vec3 ray_inv_dir;
        ray_inv_dir.x = (fabsf(ray_dir.x) < SIMD_EPSILON) ? LARGE_FLOAT : 1.0f / ray_dir.x;
        ray_inv_dir.y = (fabsf(ray_dir.y) < SIMD_EPSILON) ? LARGE_FLOAT : 1.0f / ray_dir.y;
        ray_inv_dir.z = (fabsf(ray_dir.z) < SIMD_EPSILON) ? LARGE_FLOAT : 1.0f / ray_dir.z;

        unsigned int sign[3] = {
            ray_inv_dir.x < 0.0f ? 1u : 0u,
            ray_inv_dir.y < 0.0f ? 1u : 0u,
            ray_inv_dir.z < 0.0f ? 1u : 0u
        };

        Vec3 ray_aabb_min = min_vec(origin, target);
        Vec3 ray_aabb_max = max_vec(origin, target);

        unsigned short query_min[3], query_max[3];
        quantise_with_clamp(query_min, ray_aabb_min, false);
        quantise_with_clamp(query_max, ray_aabb_max, true);

        if (num_subtree_headers > 0) {
            for (int i = 0; i < num_subtree_headers; i++) {
                const BvhSubtreeInfo& subtree = subtree_headers[i];

                bool overlap = test_quantised_aabb_overlap(
                    query_min, query_max,
                    subtree.quantised_aabb_min, subtree.quantised_aabb_max
                );

                if (overlap) {
                    walk_stackless_quantised_tree_ray(
                        cb, origin, ray_inv_dir, sign,
                        query_min, query_max,
                        lambda_max, t_max,
                        subtree.root_node_index,
                        subtree.root_node_index + subtree.subtree_size
                    );
                }
            }
        } else {
            walk_stackless_quantised_tree_ray(
                cb, origin, ray_inv_dir, sign,
                query_min, query_max,
                lambda_max, t_max,
                0, num_nodes
            );
        }
    }
};

RS_INLINE Aabb update_triangle_aabb(Aabb aabb) {
    Vec3 diff = aabb.max - aabb.min;

    if (diff.x < MIN_AABB_DIMENSION) {
        aabb.max.x += MIN_AABB_HALF_DIMENSION;
        aabb.min.x -= MIN_AABB_HALF_DIMENSION;
    }
    if (diff.y < MIN_AABB_DIMENSION) {
        aabb.max.y += MIN_AABB_HALF_DIMENSION;
        aabb.min.y -= MIN_AABB_HALF_DIMENSION;
    }
    if (diff.z < MIN_AABB_DIMENSION) {
        aabb.max.z += MIN_AABB_HALF_DIMENSION;
        aabb.min.z -= MIN_AABB_HALF_DIMENSION;
    }

    return aabb;
}

struct Bvh {
    Aabb aabb;
    int cur_node_index;
    BvhNode* nodes;
    int num_nodes;

    RS_INLINE Bvh() : cur_node_index(0), nodes(nullptr), num_nodes(0) {}

    RS_INLINE void swap_leaf_nodes(BvhNode* leaf_nodes, int i, int split_index) {
        BvhNode tmp = leaf_nodes[i];
        leaf_nodes[i] = leaf_nodes[split_index];
        leaf_nodes[split_index] = tmp;
    }

    RS_INLINE Vec3 get_aabb_centre(const BvhNode& node) const {
        return (node.aabb.max + node.aabb.min) * 0.5f;
    }

    int calc_splitting_axis(BvhNode* leaf_nodes, int start_index, int end_index) {
        int num_indices = end_index - start_index;
        Vec3 means = Vec3::zero();
        Vec3 variance = Vec3::zero();

        for (int i = start_index; i < end_index; i++) {
            Vec3 centre = get_aabb_centre(leaf_nodes[i]);
            means = means + centre;
        }
        means = means * (1.0f / (float)num_indices);

        for (int i = start_index; i < end_index; i++) {
            Vec3 centre = get_aabb_centre(leaf_nodes[i]);
            Vec3 diff = centre - means;
            Vec3 diff2(diff.x * diff.x, diff.y * diff.y, diff.z * diff.z);
            variance = variance + diff2;
        }
        variance = variance * (1.0f / ((float)num_indices - 1.0f));

        return max_variance_axis(variance);
    }

    int sort_and_calc_splitting_index(BvhNode* leaf_nodes, int start_index, int end_index, int split_axis) {
        int num_indices = end_index - start_index;
        float split_value;

        Vec3 means = Vec3::zero();
        for (int i = start_index; i < end_index; i++) {
            Vec3 centre = get_aabb_centre(leaf_nodes[i]);
            means = means + centre;
        }
        means = means * (1.0f / (float)num_indices);

        float means_arr[3] = { means.x, means.y, means.z };
        split_value = means_arr[split_axis];

        int split_index = start_index;
        for (int i = start_index; i < end_index; i++) {
            Vec3 centre = get_aabb_centre(leaf_nodes[i]);
            float centre_arr[3] = { centre.x, centre.y, centre.z };
            if (centre_arr[split_axis] > split_value) {
                swap_leaf_nodes(leaf_nodes, i, split_index);
                split_index++;
            }
        }

        int range_balanced_indices = num_indices / 3;
        bool unbalanced = ((split_index <= (start_index + range_balanced_indices)) ||
                          (split_index >= (end_index - 1 - range_balanced_indices)));
        if (unbalanced) {
            split_index = start_index + (num_indices >> 1);
        }

        return split_index;
    }

    void build_tree_recursive(BvhNode* leaf_nodes, int start_index, int end_index) {
        int num_indices = end_index - start_index;
        int cur_index = cur_node_index;

        if (num_indices == 1) {
            nodes[cur_node_index] = leaf_nodes[start_index];
            cur_node_index++;
            return;
        }

        int split_axis = calc_splitting_axis(leaf_nodes, start_index, end_index);
        int split_index = sort_and_calc_splitting_index(leaf_nodes, start_index, end_index, split_axis);

        int internal_node_index = cur_node_index;

        nodes[internal_node_index].aabb.min = aabb.max;
        nodes[internal_node_index].aabb.max = aabb.min;

        for (int i = start_index; i < end_index; i++) {
            nodes[internal_node_index].aabb = nodes[internal_node_index].aabb.merge(leaf_nodes[i].aabb);
        }

        nodes[internal_node_index].node_type = NODE_BRANCH;
        cur_node_index++;

        build_tree_recursive(leaf_nodes, start_index, split_index);
        build_tree_recursive(leaf_nodes, split_index, end_index);

        int escape_index = cur_node_index - cur_index;
        nodes[internal_node_index].escape_index = escape_index;
    }

    void build(BvhNode* leaf_nodes, int num_leaf_nodes, Aabb root_aabb) {
        aabb = root_aabb;
        cur_node_index = 0;
        num_nodes = 2 * num_leaf_nodes;
        nodes = new BvhNode[num_nodes];

        build_tree_recursive(leaf_nodes, 0, num_leaf_nodes);

        num_nodes = cur_node_index;
    }

    RS_INLINE bool check_overlap_with(const Aabb& test) const {
        if (!test.intersects(aabb)) return false;

        int cur_index = 0;
        while (cur_index < num_nodes) {
            const BvhNode& node = nodes[cur_index];
            bool overlap = test.intersects(node.aabb);

            if (node.node_type == NODE_LEAF) {
                if (overlap) return true;
                cur_index++;
            } else {
                cur_index += overlap ? 1 : node.escape_index;
            }
        }
        return false;
    }

    template<typename Callback>
    RS_INLINE void report_aabb_overlapping_node(Callback& cb, const Aabb& test) const {
        if (!test.intersects(aabb)) return;

        int cur_index = 0;
        while (cur_index < num_nodes) {
            const BvhNode& node = nodes[cur_index];
            bool overlap = test.intersects(node.aabb);

            if (node.node_type == NODE_LEAF) {
                if (overlap) {
                    cb.process_node(node.triangle_index);
                }
                cur_index++;
            } else {
                cur_index += overlap ? 1 : node.escape_index;
            }
        }
    }

    RS_INLINE bool ray_aabb_intersect(const Vec3& origin, const Vec3& inv_dir, const Aabb& aabb, float t_max) const {
        float t1 = (aabb.min.x - origin.x) * inv_dir.x;
        float t2 = (aabb.max.x - origin.x) * inv_dir.x;
        float t3 = (aabb.min.y - origin.y) * inv_dir.y;
        float t4 = (aabb.max.y - origin.y) * inv_dir.y;
        float t5 = (aabb.min.z - origin.z) * inv_dir.z;
        float t6 = (aabb.max.z - origin.z) * inv_dir.z;

        float tmin = fmaxf(fmaxf(fminf(t1, t2), fminf(t3, t4)), fminf(t5, t6));
        float tmax = fminf(fminf(fmaxf(t1, t2), fmaxf(t3, t4)), fmaxf(t5, t6));

        return tmax >= 0 && tmin <= tmax && tmin <= t_max;
    }

    template<typename Callback>
    RS_INLINE void report_ray_overlapping_node(Callback& cb, const Vec3& origin, const Vec3& direction, float& t_max) const {
        if (t_max <= 0) return;

        Vec3 inv_dir(
            fabsf(direction.x) > SIMD_EPSILON ? 1.0f / direction.x : (direction.x >= 0 ? 1e18f : -1e18f),
            fabsf(direction.y) > SIMD_EPSILON ? 1.0f / direction.y : (direction.y >= 0 ? 1e18f : -1e18f),
            fabsf(direction.z) > SIMD_EPSILON ? 1.0f / direction.z : (direction.z >= 0 ? 1e18f : -1e18f)
        );

        if (!ray_aabb_intersect(origin, inv_dir, aabb, t_max)) return;

        int cur_index = 0;
        while (cur_index < num_nodes) {
            const BvhNode& node = nodes[cur_index];
            bool overlap = ray_aabb_intersect(origin, inv_dir, node.aabb, t_max);

            if (node.node_type == NODE_LEAF) {
                if (overlap) cb.process_ray_node(node.triangle_index, t_max);
                cur_index++;
            } else {
                cur_index += overlap ? 1 : node.escape_index;
            }
        }
    }
};
