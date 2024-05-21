#ifndef BUFF_WINDOW_HPP_
#define BUFF_WINDOW_HPP_

//C++
#include <iterator>
#include <memory>
#include <string>
#include <vector>
#include <iostream>


#include <openvino/openvino.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>

#include <opencv2/core.hpp>
#include <openvino/openvino.hpp>
#include "buff_detector/param_struct.hpp"

namespace rm_buff
{
    struct BuffObject : Object
    {
        cv::Point2f apex[5];
    };

    struct GridAndStride
    {
        int grid0;
        int grid1;
        int stride;
    };
    

    class BuffDetector
    {
        public:
            BuffDetector();
            ~BuffDetector();
            bool detect(cv::Mat &src, std::vector<BuffObject>& objects);
            bool initModel(std::string path);
        private:
            int dw, dh;
            float rescale_ratio;

            ov::Core core;
            std::shared_ptr<ov::Model> model; // 网络
            ov::CompiledModel compiled_model; // 可执行网络
            ov::InferRequest infer_request;   // 推理请求
            ov::Tensor input_tensor;
            
            std::string input_name;
            std::string output_name;
            
            Eigen::Matrix<float,3,3> transfrom_matrix;
    };
}

#endif