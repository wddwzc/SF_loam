import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

# 0.1  0.12  0.15  0.18  0.2  0.25  0.3  0.35  0.4  0.5
make_time1_5 = [3.68, 3.72, 3.69, 3.61, 3.76, 3.72, 3.87, 3.68, 3.7, 3.67]
match_time1_5 = [2.14, 2.2, 2.15, 2.22, 2.17, 2.12, 2.3, 2.15, 2.18, 2.14]
P_1_5 = [1, 1, 1, 0.985, 0.898, 0.808, 0.6, 0.42, 0.275, 0.212, 0.181]
R_1_5 = [0, 0.733, 0.817, 0.873, 0.901, 0.923, 0.95, 0.959, 0.971, 0.988, 1]

make_time6_5 = [4.4, 4.42, 4.03, 4.02, 3.62, 3.5, 4.04, 3.82, 3.69, 3.77]
match_time6_5 = [2.26, 4.42, 2.23, 2.2, 1.94, 1.87, 2.09, 2.08, 1.99, 2.05]
P_6_5 = [1, 1, 1, 1, 1, 0.998, 0.927, 0.636, 0.365, 0.258, 0.19]
R_6_5 = [0, 0.575, 0.641, 0.708, 0.774, 0.804, 0.861, 0.891, 0.929, 0.956, 0.998]

make_time7_5 = [3.98, 3.64, 3.76, 3.48, 3.65, 3.56, 3.48, 3.82, 3,74, 3.79]
match_time7_5 = [2.02, 1.86, 1.93, 1.73, 1.85, 1.83, 1,75, 2, 1.92, 1.98]
P_7_5 = [1, 1, 1, 1, 1, 1, 1, 0.985, 0.83, 0.671, 0.34]
R_7_5 = [0, 0.659, 0.763, 0.796, 0.859, 0.9, 0.947, 0.989, 0.993, 0.996, 1]

make_time9_5 = [4.08, 3.91, 3.52, 3.42, 3.56, 3.58, 3.45, 3.43, 3.43, 3.44, 3.43]
match_time9_5 = [2.25, 2.14, 1.96, 1.91, 1.99, 2.01, 1.91, 1.91, 1.91, 1.91, 1.91]
P_9_5 = [1, 1, 1, 1, 0.938, 0.647, 0.383, 0.238, 0.154, 0.121, 0.0911]
R_9_5 = [0, 0.043, 0.14, 0.292, 0.447, 0.504, 0.688, 0.777, 0.862, 0.931, 0.989]

make_time12_5 = [1.75, 1.76, 1.79, 1.76, 1.75, 1.76, 1.75, 1.76, 1.76]
match_time12_5 = [2.04, 2.05, 2.09, 2.04, 2.04, 2.04, 2.03, 2.03, 2.04]
P_12_5 = [1, 1, 1, 1, 0.985, 0.765, 0.396, 0.206, 0.107, 0.0721, 0.0422]
R_12_5 = [0, 0.13, 0.24, 0.412, 0.622, 0.695, 0.763, 0.821, 0.847, 0.866, 0.882]

make_time15_5 = [1.9, 1.8, 1.79, 1.8, 1.81, 1.8, 1.79, 1.79, 1.91, 1.8]
match_time15_5 = [2.13, 1.97, 1.90, 1.97, 1.99, 1.98, 1.96, 1.95, 2.19, 1.97]
P_15_5 = [1, 1, 1, 0.995, 0.989, 0.968, 0.872, 0.872, 0.77, 0.469, 0.207]
R_15_5 = [0, 0.19, 0.353, 0.492, 0.557, 0.581, 0.671, 0.801, 0.906, 0.963, 0.995]



# 0.1  0.12  0.15  0.18  0.2  0.25  0.3  0.35  0.4  0.5
P_1_10 = [1, 1, 1, 0.984, 0.898, 0.808, 0.615, 0.438, 0.290, 0.230, 0.208]
R_1_10 = [0, 0.651, 0.727, 0.776, 0.801, 0.822, 0.866, 0.890, 0.911, 0.942, 0.995]

P_6_10 = [1, 1, 1, 1, 1, 0.997, 0.933, 0.644, 0.371, 0.271, 0.230]
R_6_10 = [0, 0.499, 0.555, 0.614, 0.671, 0.697, 0.752, 0.783, 0.817, 0.860, 0.993]

P_7_10 = [1, 1, 1, 1, 1, 1, 1, 0.985, 0.829, 0.673, 0.359]
R_7_10 = [0, 0.649, 0.751, 0.784, 0.846, 0.886, 0.959, 0.974, 0.978, 0.985, 1]

P_9_10 = [1, 1, 1, 1, 0.838, 0.647, 0.383, 0.248, 0.167, 0.135, 0.106]
R_9_10 = [0, 0.036, 0.120, 0.25, 0.382, 0.431, 0.588, 0.691, 0.801, 0.897, 0.980]

P_12_13 = [1, 1, 1, 1, 0.988, 0.773, 0.41, 0.223, 0.145, 0.145, 0.123]
R_12_13 = [0, 0.0457, 0.084, 0.145, 0.22, 0.247, 0.277, 0.313, 0.4059, 0.616, 0.893]

P_15_10 = [1, 1, 1, 1, 1, 0.989, 0.904, 0.881, 0.766, 0.474, 0.223]
R_15_10 = [0, 0.177, 0.328, 0.46, 0.524, 0.553, 0.648, 0.754, 0.841, 0.906, 0.977]


make_time1_5_array = np.array(make_time1_5)
match_time1_5_array = np.array(match_time1_5)
make_time6_5_array = np.array(make_time6_5)
match_time6_5_array = np.array(match_time6_5)
make_time7_5_array = np.array(make_time7_5)
match_time7_5_array = np.array(match_time7_5)
make_time9_5_array = np.array(make_time9_5)
match_time9_5_array = np.array(match_time9_5)
make_time12_5_array = np.array(make_time12_5)
match_time12_5_array = np.array(match_time12_5)
make_time15_5_array = np.array(make_time15_5)
match_time15_5_array = np.array(match_time15_5)

print(make_time1_5_array.mean())
print(match_time1_5_array.mean())
print(make_time6_5_array.mean())
print(match_time6_5_array.mean())
print(make_time7_5_array.mean())
print(match_time7_5_array.mean())
print(make_time9_5_array.mean())
print(match_time9_5_array.mean())
print(make_time12_5_array.mean())
print(match_time12_5_array.mean())
print(make_time15_5_array.mean())
print(match_time15_5_array.mean())




# plt.figure(figsize=(7,5))
# plt.plot(R_1_10, P_1_10, 'ro-', lw=1.5, label='SC')
# plt.plot(R_1_5, P_1_5, 'bo-', lw=1.5, label='SISC')
# plt.grid(True)
# plt.legend(loc=1)
# plt.xlim(0, 1.1)
# plt.ylim(0, 1.1)
# #plt.axis('tight')
# plt.xlabel('Recall')
# plt.ylabel('Precision')
# plt.title('NO.1 ROC curve')
# plt.show()


# plt.figure(figsize=(7,5))
# plt.plot(R_6_10, P_6_10, 'ro-', lw=1.5, label='SC')
# plt.plot(R_6_5, P_6_5, 'bo-', lw=1.5, label='SISC')
# plt.grid(True)
# plt.legend(loc=1)
# plt.xlim(0, 1.1)
# plt.ylim(0, 1.1)
# #plt.axis('tight')
# plt.xlabel('Recall')
# plt.ylabel('Precision')
# plt.title('NO.6 ROC curve')
# plt.show()


# plt.figure(figsize=(7,5))
# plt.plot(R_7_10, P_7_10, 'ro-', lw=1.5, label='SC')
# plt.plot(R_7_5, P_7_5, 'bo-', lw=1.5, label='SISC')
# plt.grid(True)
# plt.legend(loc=1)
# plt.xlim(0, 1.1)
# plt.ylim(0, 1.1)
# #plt.axis('tight')
# plt.xlabel('Recall')
# plt.ylabel('Precision')
# plt.title('NO.7 ROC curve')
# plt.show()


# plt.figure(figsize=(7,5))
# plt.plot(R_9_10, P_9_10, 'ro-', lw=1.5, label='SC')
# plt.plot(R_9_5, P_9_5, 'bo-', lw=1.5, label='SISC')
# plt.grid(True)
# plt.legend(loc=1)
# plt.xlim(0, 1.1)
# plt.ylim(0, 1.1)
# #plt.axis('tight')
# plt.xlabel('Recall')
# plt.ylabel('Precision')
# plt.title('NO.9 ROC curve')
# plt.show()



# plt.figure(figsize=(7,5))
# plt.plot(R_12_13, P_12_13, 'ro-', lw=1.5, label='SC')
# plt.plot(R_12_5, P_12_5, 'bo-', lw=1.5, label='SISC')
# plt.grid(True)
# plt.legend(loc=1)
# plt.xlim(0, 1.1)
# plt.ylim(0, 1.1)
# #plt.axis('tight')
# plt.xlabel('Recall')
# plt.ylabel('Precision')
# plt.title('NO.12 ROC curve')
# plt.show()




# plt.figure(figsize=(7,5))
# plt.plot(R_15_10, P_15_10, 'ro-', lw=1.5, label='SC')
# plt.plot(R_15_5, P_15_5, 'bo-', lw=1.5, label='SISC')
# plt.grid(True)
# plt.legend(loc=1)
# plt.xlim(0, 1.1)
# plt.ylim(0, 1.1)
# #plt.axis('tight')
# plt.xlabel('Recall')
# plt.ylabel('Precision')
# plt.title('NO.15 ROC curve')
# plt.show()