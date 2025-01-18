import numpy as np
import os
import matplotlib.pyplot as plt


if __name__ == '__main__':
    label_names = []
    
    # get predictions paths
    # pred_names = []
    # for sequence in test_sequences:
    #     sequence = '{0:02d}'.format(int(sequence))
    #     pred_paths = os.path.join(FLAGS.predictions, "sequences",
    #                             sequence, "predictions")
    #     # populate the label names
    #     seq_pred_names = [os.path.join(dp, f) for dp, dn, fn in os.walk(
    #         os.path.expanduser(pred_paths)) for f in fn if ".label" in f]
    #     seq_pred_names.sort()
    #     pred_names.extend(seq_pred_names)
    # # print(pred_names)

    # # check that I have the same number of files
    # print("labels: ", len(label_names))
    # print("predictions: ", len(pred_names))
    # for label_file, pred_file in zip(label_names[:], pred_names[:]):
    
    
    # label_file = "/home/huajie/Downloads/event_detection/LiDAR_MOS_toy_dataset/sequences/08/labels/" + format(num, '06d') +".label"
    # pred_file = "/home/huajie/Downloads/event_detection/LiDAR_MOS_toy_dataset/sequences/08/detect/" + format(num-1, '06d') +".txt"
    # point_file = "/home/huajie/Downloads/event_detection/LiDAR_MOS_toy_dataset/sequences/08/velodyne/" + format(num, '06d') +".bin"
    label_folder = "/home/huajie/Downloads/event_detection/data_odometry_labels/dataset/sequences/06/labels/"
    # print(os.listdir(label_folder))
    label_num = len(os.listdir(label_folder))
    
    all_num = 0
    count = 0
    every_frame = []
    t = []
    for l in range(0,label_num):
        label_file = label_folder + format(l, '06d') +".label"
        
        label = np.fromfile(label_file, dtype=np.int32)
        label = label.reshape((-1))  # reshape to vector
        
        
        cur_num = 0
        for i in range(len(label)):
            
            label[i] = label[i] & 0xFFFF
            if(label[i] > 251 and label[i] < 260): 
                cur_num += 1
        all_num += cur_num
        every_frame.append(cur_num)
        t.append(l)
        print(cur_num)
    
    print(all_num/label_num) 
    plt.plot(t, every_frame)
    plt.show()           
   
    # # when I am done, print the evaluation
    # m_accuracy = evaluator.getacc()
    # m_jaccard, class_jaccard = evaluator.getIoU()