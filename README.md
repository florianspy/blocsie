# blocsie
BLOCSIE - Benchmark for LOCalization in a Simulated Industrial Environment

Link to the unity project
https://drive.google.com/file/d/1koVoI8hh0QAry02isOlENKvnfQ_PISZm/view
The Assets from external sources are listed in 

Link to the evaluation part
https://github.com/florianspy/locchallbench/tree/main

The source code for the ros package containing the ros msg definition for transmitting angle, material, distance in one msg is in the ros_msg folder. It is required to be build before the noise generator.

The source code to create txt files used in the noise generator as a lookup table for the datadriven model can be found in modelgn folder

The source code for the noisegenerator can be found in noisegn folder, keep in mind it requires the FileHandler from modelgn folder.

