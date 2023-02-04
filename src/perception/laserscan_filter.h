//
// Created by waxz on 23-2-4.
//

#ifndef SCAN_REPUBLISHER_LASERSCAN_FILTER_H
#define SCAN_REPUBLISHER_LASERSCAN_FILTER_H


namespace perception{
    class RangesFilter{

    public:
        void add(const std::vector<float>& ranges){


            int point_num = ranges.size();
            if(!is_init){
                matrix_buffer = Eigen::MatrixXf(max_len,point_num);
//            ranges_buffer.resize(max_len, ranges);
                is_init = true;
            }
            if(update_index >=max_len){
                return;
            }

            for(int i = 0 ; i < point_num;i++){
                matrix_buffer(update_index,i) = ranges[i];
            }
//        ranges_buffer[update_index] = ranges;

            update_index++;

        }
        bool filter(){
            if(update_index < max_len){
                return false;
            }
            Eigen::VectorXf mean_vector = matrix_buffer.colwise().mean();

            Eigen::MatrixXf centered = matrix_buffer.rowwise() - mean_vector.transpose();



//        Eigen::Array<double, 1, Eigen::Dynamic> std_dev = (centered.square().colwise().sum()/(M-1)).sqrt();

            auto std_dev = centered.array().pow(2).colwise().sum() /(max_len -1);


            int point_num = std_dev.cols();

//        PLOGD << "std_dev " << std_dev << std::endl;

            ranges_filtered.resize(point_num);
            for(int i = 0 ; i <point_num;i++){

                ranges_filtered[i] = std_dev(0,i) < max_stddev ? mean_vector(i):100.0;
            }
            return true;

        }
        const std::vector<float>& getFiltered(){

            return ranges_filtered;
        }
        void clear(){
            update_index = 0;
        }

    public:
        int max_len = 10;
        float max_stddev = 0.01;
        int update_index = 0;

    private:
        bool is_init = false;
        std::vector<std::vector<float>> ranges_buffer;
        std::vector<float> ranges_filtered;
        std::vector<float> ranges_stddev;

        Eigen::MatrixXf matrix_buffer;

    };
}
#endif //SCAN_REPUBLISHER_LASERSCAN_FILTER_H
