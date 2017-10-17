#ROS HEADER
Header header

#size of measured feature vector
int16 size_s_msr

#size of  desired feature vector
int16 size_s_des

#number of features
int16 numFeatures

#row in Lmsr
int16 rowsLmsr

#col in Ldes
int16 colsLmsr

#row in Ldes
int16 rowsLdes

#col in Ldes
int16 colsLdes


# valid data current simple features flag
bool is_valid_msr_feature

# valid data desired simple feature flag
bool is_valid_des_feature



#image current simple feature matrices
float64[] s_msr

# interaction measured Matrix simple features (represented as row vector with size sizeFeatureVector x sizeFeatureVector )
float64[] L_data_msr

#image desired simple feature matrices
float64[] s_des

# interaction desired Matrix simple features(represented as row vector with size sizeFeatureVector x sizeFeatureVector )
float64[] L_data_des

