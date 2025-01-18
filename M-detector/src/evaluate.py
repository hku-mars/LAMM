import numpy as np
import os
import matplotlib.pyplot as plt

import sys
import numpy as np


class iouEval:
    def __init__(self, n_classes, ignore=None):
        # classes
        self.n_classes = n_classes

        # What to include and ignore from the means
        self.ignore = np.array(ignore, dtype=np.int64)
        self.include = np.array(
            [n for n in range(self.n_classes) if n not in self.ignore], dtype=np.int64)
        print("[IOU EVAL] IGNORE: ", self.ignore)
        print("[IOU EVAL] INCLUDE: ", self.include)

        # reset the class counters
        self.reset()

    def num_classes(self):
        return self.n_classes

    def reset(self):
        self.conf_matrix = np.zeros((self.n_classes,
                                    self.n_classes),
                                    dtype=np.int64)

    def addBatch(self, x, y):  # x=preds, y=targets
        # sizes should be matching
        x_row = x.reshape(-1)  # de-batchify
        y_row = y.reshape(-1)  # de-batchify

        # check
        assert(x_row.shape == y_row.shape)

        # create indexes
        idxs = tuple(np.stack((x_row, y_row), axis=0))

        # make confusion matrix (cols = gt, rows = pred)
        np.add.at(self.conf_matrix, idxs, 1)

    def getStats(self):
        # remove fp from confusion on the ignore classes cols
        conf = self.conf_matrix.copy()
        conf[:, self.ignore] = 0

        # get the clean stats
        tp = np.diag(conf)
        fp = conf.sum(axis=1) - tp
        fn = conf.sum(axis=0) - tp
        return tp, fp, fn

    def getIoU(self):
        tp, fp, fn = self.getStats()
        intersection = tp
        union = tp + fp + fn + 1e-15
        iou = intersection / union
        iou_mean = (intersection[self.include] / union[self.include]).mean()
        return iou_mean, iou  # returns "iou mean", "iou per class" ALL CLASSES

    def getacc(self):
        tp, fp, fn = self.getStats()
        total_tp = tp.sum()
        total = tp[self.include].sum() + fp[self.include].sum() + 1e-15
        acc_mean = total_tp / total
        return acc_mean  # returns "acc mean"
        
    def get_confusion(self):
        return self.conf_matrix.copy()



if __name__ == "__main__":
    


    
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
    num = 140
    
    # label_file = "/home/huajie/Downloads/event_detection/LiDAR_MOS_toy_dataset/sequences/08/labels/" + format(num, '06d') +".label"
    # pred_file = "/home/huajie/Downloads/event_detection/LiDAR_MOS_toy_dataset/sequences/08/detect/" + format(num-1, '06d') +".txt"
    # point_file = "/home/huajie/Downloads/event_detection/LiDAR_MOS_toy_dataset/sequences/08/velodyne/" + format(num, '06d') +".bin"
    label_folder = "/home/huajie/Downloads/event_detection/data_odometry_labels/dataset/sequences/04/labels/"
    # print(os.listdir(label_folder))detect_0.005_0.005_0.01_cluster
    label_num = len(os.listdir(label_folder))
    pred_folder = "/home/huajie/Downloads/event_detection/data_odometry_labels/dataset/sequences/04/predict_0404/"
    pred_num = len(os.listdir(pred_folder))
    print(label_num, pred_num)
    
    point_file = "/home/huajie/Downloads/event_detection/data_odometry_labels/dataset/sequences/07/velodyne/" + format(num, '06d') +".bin"
    # print(label_file)
    # print(pred_file)
    # pointcloud = np.fromfile(str(point_file), dtype=np.float32, count=-1).reshape([-1,4])
    result_file ="/home/huajie/Downloads/event_detection/data_odometry_labels/dataset/sequences/07/detect.txt"
    f = open(result_file, 'w')
 
    # print(pointcloud.shape)
    # x = pointcloud[:, 0]  # x position of point
    # y = pointcloud[:, 1]  # y position of point
    # z = pointcloud[:, 2]  # z position of point
    # r = pointcloud[:, 3]  # reflectance value of point
    # d = np.sqrt(x ** 2 + y ** 2)  # Map Distance from sensor
    all_iou = 0.0
    t = []
    ious = []
    minus_num = label_num - pred_num
    start_index = 9
    tp_zero = 0
    for l in range(start_index,label_num):
        label_file = label_folder + format(l, '06d') +".label"
        pred_file = pred_folder + format(l-minus_num, '06d') +".txt"
        label = np.fromfile(label_file, dtype=np.int32)
        label = label.reshape((-1))  # reshape to vector
        # label = label & 0xFFFF       # get lower half for semantics
        # if FLAGS.limit is not None:
        #     label = label[:FLAGS.limit]  # limit to desired length
        #     label = remap_lut[label]       # remap to xentropy format
        # print(len(label))
        # print(label)
        # open prediction
        pred = np.loadtxt(pred_file)
        pred = pred.reshape((-1))    # reshape to vector
        # pred = pred & 0xFFFF         # get lower half for semantics
        # if FLAGS.limit is not None:
        #     pred = pred[:FLAGS.limit]  # limit to desired length
        #     pred = remap_lut[pred]       # remap to xentropy format
        # print(label_file)
        # print(pred_file)
        # print(len(pred))
        assert(len(label) == len(pred))
        
        
        # tp = fp = fn = 0
        # count = 0
        
        # for i in range(len(pred)):
        #     # if(pred[i] == 1):
        #     #     print("1111")
        #     # if(label[i] > 1 and label[i] != 40 and label[i] != 48 and label[i] != 99 and label[i] != 70 and label[i] != 72 and label[i] != 71 and label[i] != 80):
        #     #     print(label[i])
        #     label[i] = label[i] & 0xFFFF
        #     if(label[i] >= 251 and label[i] < 260):
        #         count += 1
        #         if(pred[i] == 1):
        #             tp += 1
        #         else:
        #             fn += 1
        #     else:
        #         if(pred[i] == 1):
        #             fp += 1
        #             # print(i)
        # # print((label[59414]))
        # print(tp, fp, fn, count)
        # if count==0:
        #     tp_zero += 1
        #     iou = 0
        # else :
        #     # tp = tp + 12
        #     intersection = tp
        #     fn = fn - 12
        #     union = tp + fp + fn + 1e-15
        #     iou = intersection / union
        # all_iou += iou
        # print(iou)
        # t.append(l)
        # ious.append(iou)
        # f.write('%d %d %d %d %f\n'%(tp, fp, fn, count, iou))
        # add single scan to evaluation
    #     evaluator.addBatch(pred, label)
    # f.close()
    print(all_iou/(label_num-start_index-tp_zero))
    plt.plot(t, ious)
    plt.show()

    # # when I am done, print the evaluation
    # m_accuracy = evaluator.getacc()
    # m_jaccard, class_jaccard = evaluator.getIoU()