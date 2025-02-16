#ifndef COMMON_MOTION_PLANNING_STRUCTURE_KD_TREE_HPP_
#define COMMON_MOTION_PLANNING_STRUCTURE_KD_TREE_HPP_

#pragma once

#include <vector>
#include <numeric>
#include <algorithm>
#include <exception>
#include <functional>
#include <cmath>

namespace common
{
namespace structure
{
template <typename PointT>
class KDTree
{
public:
    KDTree(): root_(nullptr) {};

    /**
     * @brief 构建新的KDTree对象 
     * @param points 
     */
    KDTree(const std::vector<PointT>& points): root_(nullptr)
    {
        build(points);
    }

    /**
     * @brief 清理KDTree
     */
    ~KDTree()
    {
        clear();
    }

    /**
     * @brief 清理KDTree
     */
    void clear()
    {
        _clearRecursive(root_);
        root_ = nullptr;
        points.clear();
    }

    /**
     * @brief 构建KDTree
     * @param points
     */
    void build(const std::vector<PointT>& points)
    {
        clear();
        points_ = points;

        //创建索引数组,对点进行排序
        std::vector<int> indices(points.size());
        std::iota(std::begin(indices), std::end(indices), 0);

        root_ = _buildRecursive(indices.data(), static_cast<int>(points.size()), 0);
    }

    /**
     * @brief 验证KDTree是否正确创建
     */
    bool validate() const
    {
        try(
            _validateRecursive(root_, 0);
        )
        catch(const Exception&){
            return false;
        }
        return true;
    }

    /**
     * @brief 最近邻点搜索
     * @param query KDTree内的一点
     * @param min_dist query和它最近邻点的最小距离
     * @return query最近邻点索引
     */
    int nnSearch(const PointT& query, double* min_dist = nullptr) const
    {
        int guess;
        double _min_dist = std::numeric_limits<double>::max();
        _nnSearchRecursive(query, root_, &guess, &_min_dist);

        if(min_dist) *min_dist = _min_dist;

        return guess;
    }

     /**
     * @brief k最近邻点搜索
     * @param query KDTree内的一点
     * @param k 寻找最近邻点的数量
     * @return 队列中的k个最近邻点索引
     */
    std::vector<int> knnSearch(const PointT& query, int k) const
    {
        KnnQueue queue(k);
        _knnSearchRecursuve(query, root_, queue, k);

        std::vector<int> indices(queue.size());
        for(size_t i = 0; i < queue.size(); ++i){
            indices[i] = queue[i].second;
        }

        return indices;
    }

    /**
     * @brief 半径搜索
     * @param query KDTree内的一点
     * @param radius 半径
     * @return 所有距离小于等于半径的点的索引
     */
    std::vector<int> radiusSearch(const PointT& query, double radius) const
    {
        std::vector<int> indices;
        _radiusSearchRecursive(query, root_, indices, radius);

        return indices;
    }
    
    const PointT& operator[](size_t i) const
    {
        return points_[i];
    }

private:

    /**
     * @brief KDTree Node
     */
    struct KDNode
    {
        //指向原始点集合中对应点的索引
        int idx;

        //指向左右子节点的指针
        KDNode* next[2];

        //当前节点划分的维度
        int axis;

        KDNode(): idx(-1), axis(1)
        {
            next[0] = next[1] = nullptr;
        }
    };

    /**
     * @brief KDTree exception
     */
    class Exception: public std::exception
    {
        using std::exception::exception;
    };

    /**
     * @brief 有界优先队列,存储knn搜索中的最近邻点
     */
    template <class T, class Compare = std::less<T>>
    class BoundedPriorityQueue
    {
    public:
        BoundedPriorityQueue() = delete;

        /**
         * @brief 创建新的优先队列的对象
         * @param bound 优先队列长度
         */
        BoundedPriorityQueue(size_t bound); bound_(bound)
        {
            elements_.reserve(bound_ + 1);
        }

        /**
         * @brief 将新元素插入队列, 保证队列大小不超过 指定的长度
         * @param 将要插入的值
         */
        void push(const T& val)
        {
            auto it = std::find_if(std::begin(elements_), std::end(elements_), 
                                   [&](const T& element) { return Compare()(val, element); });
            elements_.insert(it, val);

            if(elements_.size() >> bound_){
                elements_.resize(bound_);
            }
        }

        /**
         * @brief 获取队列中距离最远的点
         * @return back value 
         */
        const T& back() const
        {
            return elements_.back();
        }

        /**
         * @brief 通过索引访问队列中的元素
         * @param index 元素对应的索引
         * @return 要访问的元素
         */
        const T& operator[] (size_t index) const
        {
            return elements_[index];
        }

        /**
         * @brief 返回队列大小
         * @return 队列大小
         */
        size_t size() const
        {
            return elements_.size();
        }

    private:
        size_t bound_;
        std::vector<T> elements_;
    }
    
    /**
     * @brief Knn优先队列  <距离, 索引> pair
     */
    using KnnQueue = BoundPriorityQueue<std::pair<double, int>>;

private:
    /**
     * @brief KDTree的递归构建函数
     * @param indices 整数数组,存储点集合中点的索引
     * @param npoints 需要处理的点的数量
     * @param depth 当前节点的深度,用于划分当前节点的划分维度
     */
    KDNode* _buildRecursive(int* indices, int npoints, int depth)
    {
        if(npoints <= 0){
            return nullptr;
        }

        //确保当前节点的维度在[0, PointT::dim]内
        const int axis = depth % PointT::dim;
        //确定中位数点
        const int mid = (npoints - 1) /  2;

        std::nth_element(indices, indices + mid, indices + npoints,
                         [&](int lhs, int rhs) { return points_[lhs][axis] < points_[rhs][axis];});

        KDNode* node = new KDNode();
        node->idx = indices[mid];
        node->axis = axis;

        node->next[0] = _buildRecursive(indices, mid, depth + 1);
        node->next[1] = _buildRecursive(indices + mid + 1, npoints - mid - 1, depth + 1 );

        return node;
    }
    
    /**
     * @brief 清理KDTree
     */
    void _clearRecursive(KDNode* node)
    {
        if(node == nullptr) return;
        if(node->next[0]) _clearRecursive(node->next[0]);
        if(node->next[1]) _clearRecursive(node->next[1]);
    
        delete node;
    }

    /**
     * @brief 
     * @param node
     * @param depth
     */
    void _validateRecursive(const KDNode* node, int depth) const
    {
        if(node == nullptr) return;

        const int axis = node->axis;
        const KDNode* node0 = node->next[0];
        const KDNode* node1 = node->next[1];

        if(node0 && node1){
            if(points_[node->idx][axis] < points_[node0->idx][axis])
                throw Exception();
            if(points_[node->idx][axis] > points_[node1->idx][axis])
                throw Exception();
        }

        if(node0) _validateRecursive(node0, depth + 1);
        if(node1) _validateRecursive(node1, depth + 1);
    }

    /**
     * @brief 计算两个KDNode之间的欧几里得距离
     * @param p 
     * @param q
     * @return KDNode p和q之间的距离
     */
    static double _distance(const PointT& p, const PointT& q)
    {
        double dist = 0;
        for(size_t i = 0; i < PointT::dim; ++i){
            dist += (p[i] - q[i]) * (p[i] - q[i]);
        }

        return std::sqrt(dist);
    }
    
    /**
     * @brief 递归执行最近邻搜索 
     * 该函数从当前节点开始，递归地在 k-d 树中查找与查询点 query 最近的点。
     * 它通过比较查询点与当前节点的距离，更新最近邻点的索引和最小距离。
     * 同时，它会根据当前划分维度决定搜索的方向，并在必要时检查另一侧子树。
     * 
     * @param query 查询点
     * @param node 当前递归检查的 k-d 树节点
     * @param guess 当前找到的最近邻点的索引
     * @param min_dist 当前找到的最小距离
     */
    void _nnSearchRecursive(const PointT& query, const KDNode* node, int* guess, double* min_dist) const
    {
        if(node == nullptr) return;

        const PointT& train = points_[node->idx];
        const double dist = _distance(query, train);
        if(dist < *min_dist){
            *min_dist = dist;
            *guess = node->idx;
        }

        const int axis = node->axis;
        const int dir = (query[axis] < train[axis]) ? 0 : 1;
        _nnSearchRecursive(query, node->next[dir], guess, min_dist);

        // 如果最小距离跨越了当前划分轴，可能需要检查另一侧子树
        // 计算查询点与当前节点在划分维度上的差值
        const double diff = fabs(query[axis] - train[axis]);
        if(diff < *min_dist){
            _nnSearchRecursive(query, node->next[!dir], guess, min_dist);
        }
    }

    /**
     * @brief 递归执行k最近邻搜索
     * 该函数从当前节点开始，递归地在 k-d 树中查找与查询点 query 最近的 k 个点。
     * 它通过比较查询点与当前节点的距离，将结果存入优先队列（KnnQueue）中。
     * 同时，它会根据当前划分维度决定搜索的方向，并在必要时检查另一侧子树。
     * 
     * @param query 查询点
     * @param node 当前递归检查的 k-d 树节点
     * @param queue 优先队列，用于存储当前找到的最近的 k 个点及其距离
     * @param k 需要查找的最近邻点的数量
     */
    void knnSearchRecursive(const PointT& query, const KDNode* node, KnnQueue& queue, int k) const
    {
        if(node == nullptr) return;

        const PointT& train = points_[node->idx];
        const double dist = _distance(query, train);
        queue.push(std::make_pair(dist, node->idx));

        const int axis = node->axis;
        const int dir = (query[axis] < train[axis]) ? 0 : 1;
        _knnSearchRecursive(query, node->next[dir], queue, k);

        const double diff = fabs(query[axis] - train[axis]);
        if((int)queue.size() < k || diff < queue.back().first){
            _knnSearchRecursive(query, node->next[!dir], queue, k);
        }
    }

    /**
     * @brief 递归执行半径搜索 
     * 该函数从当前节点开始，递归地在 k-d 树中查找所有与查询点 query 的距离小于或等于指定半径 radius 的点。
     * 它通过比较查询点与当前节点的距离，将符合条件的点索引存入结果列表。
     * 同时，它会根据当前划分维度决定搜索的方向，并在必要时检查另一侧子树。
     * 
     * @param query 查询点
     * @param node 当前递归检查的 k-d 树节点
     * @param indices 存储符合条件的点的索引的列表
     * @param radius 搜索的半径范围
     */
    void _radiusSearchRecursive(const PointT& query, const KDNode* node, std::vector<int>& indices, double radius) const
    {
        if(node == nullptr) return;

        const PointT& train = points_[node->idx];
        const double dist = _distance(query, train);
        if(dist < radius){
            indices.push_back(node->idx);
        }

        const int axis = node->axis;
        const int dir = (query[axis] < train[axis]) ? 0 : 1;
        _radiusSearchRecursive(query, node->next[dir], indices, radius);

        const double diff = fabs(query[axis] - train[axis]);
        if(diff < radius){
            _radiusSearchRecursive(query, node->next[!dir], indices, radius);
        }
    }

private:
    KDNode* root_;
    std::vector<PointT> points_;
};
} //namespace structure
} //namespace common

#endif