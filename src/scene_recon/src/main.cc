#include <cstdio>
#include <iostream>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <map>
#include "controllers/automatic_reconstruction.h"
#include <gflags/gflags.h>

DEFINE_string(image_path, "", "");
DEFINE_string(vocab_tree_path, "", "");
DEFINE_string(workspace_path, "", "");

int main(int argc, char **argv){

    google::ParseCommandLineFlags(&argc, &argv, true);

    colmap::AutomaticReconstructionController::Options options;
    options.workspace_path=FLAGS_workspace_path;
    options.image_path=FLAGS_image_path;
    options.vocab_tree_path=FLAGS_vocab_tree_path;
    options.data_type = colmap::AutomaticReconstructionController::DataType::INDIVIDUAL;
    options.quality = colmap::AutomaticReconstructionController::Quality::HIGH;
    options.single_camera = true;
    options.camera_model = "SIMPLE_RADIAL";
    options.sparse = true;
    options.dense = true;
    options.mesher = colmap::AutomaticReconstructionController::Mesher::POISSON;
    options.num_threads = -1;
    options.use_gpu = true;
    options.gpu_index = "-1";
    colmap::ReconstructionManager* reconstruction_manager;
    colmap::AutomaticReconstructionController app(options, reconstruction_manager);
//     app.Run();

    return 0;
}
