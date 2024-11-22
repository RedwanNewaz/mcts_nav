//
// Created by airlab on 11/18/24.
//

#ifndef MCTS_TRAIN_H
#define MCTS_TRAIN_H
#include <iostream>
#include <string>

namespace base{
    class train{
    public:
        train(int num_epochs, const std::string& outfile=""):
        num_epochs_(num_epochs), output_file_(outfile)
        {

        }
        void run()
        {
            double total_reward = 0;
            for (int epoch = 1; epoch <= num_epochs_; ++epoch) {
                double step_reward = step(epoch);
                total_reward += step_reward ;
                printf("[>] epoch = %d, step reward = %lf,  avg reward = %lf \n", epoch, step_reward, total_reward / epoch);
            }

            if(!output_file_.empty())
                save(output_file_);
        }
    protected:
        int num_epochs_;
        std::string output_file_;
        virtual void save(const std::string& outfile) = 0;
        virtual double step(int epoch) = 0;
    };
}
#endif //MCTS_TRAIN_H
