#!/usr/bin/env python3

import pathlib
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
import numpy as np
import tqdm
import sys
import datetime

pkg_dir = pathlib.Path(__file__).parent.parent

# 位置と法線ベクトルの分散
def extract_feature(position_list, normal_list):
    feature = []
    for position, normal in zip(position_list, normal_list):
        position_var = np.std(position, axis=0)
        normal_var = np.std(normal, axis=0)
        feature.append(np.concatenate([position_var.flatten(), normal_var.flatten()]))
    return feature

class Net(nn.Module):
    def __init__(self, input_size, hidden_size, output_size):
        super(Net, self).__init__()
        self.fc1 = nn.Linear(input_size, hidden_size)
        self.fc2 = nn.Linear(hidden_size, output_size)
    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = self.fc2(x)
        return x

class MyDataset(Dataset):
    def __init__(self, LabelList):
        dataset_dir = pkg_dir/"datasets"
        self.label_list = []
        position_list = []
        normal_list = []
        # フォルダ名
        for path in dataset_dir.glob("**/*"):
            if path.is_file():
                with open(path) as f:
                    txt_list = f.read().split("\n")
                    for txt in txt_list[:-1]:
                        txt = np.array(txt.split(","))
                        if txt[0]=="unknown":
                            self.label_list.append(0)
                        else:
                            for i,label in enumerate(LabelList):
                                if txt[0]==label:
                                    self.label_list.append(i+1)
                                else:
                                    self.label_list.append(0)
                        num_node = int(len(txt[1:])/6)
                        pos = np.array([[float(txt[1+6*i+j]) for j in range(3)] for i in range(num_node)])
                        normal = np.array([[float(txt[1+6*i+3+j]) for j in range(3)] for i in range(num_node)])
                        position_list.append(pos)
                        normal_list.append(normal)
        self.feature = extract_feature(position_list, normal_list)
    
    def __len__(self):
        return len(self.feature)
    
    def __getitem__(self, index):
        return torch.tensor(self.feature[index], dtype=torch.float32), torch.tensor(self.label_list[index], dtype=torch.long)

if __name__ == "__main__":
    label_to_learn = "human"# 学習対象のラベル
    
    # コマンド引数からラベルを取得
    args = sys.argv
    if len(args) > 1:
        if args[1] == "human" or args[1] == "car":
            label_to_learn = args[1]

    print("Learning label:", label_to_learn)
    
    device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')

    ds = MyDataset([label_to_learn])
    
    print("Number of data:", len(ds))
    
    if len(ds) == 0:
        print("No data found for the specified label.")
        sys.exit(1)
    
    data_loader = DataLoader(ds, batch_size=1, shuffle=True, num_workers=4, pin_memory=True)

    net = Net(ds.feature[0].shape[0], 8, 3)
    best_model = net
    best_loss = 1e6

    loss_func = nn.CrossEntropyLoss()
    optimizer = optim.Adam(net.parameters())

    print(ds.feature[0].shape[0], np.shape(ds.label_list))

    for epoch in range(1, 501):
        loss_ave_train = 0
        net.train()
        for data, label in tqdm.tqdm(data_loader):
            optimizer.zero_grad()
            out = net(data)
            loss = loss_func(out, label)
            loss.backward()
            optimizer.step()
            loss_ave_train += loss.item()
        loss_ave_train /= len(ds)
        if (best_loss  > loss_ave_train):
            print("Best!")
            best_loss = loss_ave_train
            best_model = net
        print(epoch, loss_ave_train)

    best_model.eval()
    # ダミーの入力データを作成
    example_input = torch.rand(ds.feature[0].shape[0])
    # print(ds.feature[0].shape[0]);quit()
    # モデルをTorchScript形式に変換
    scripted_model = torch.jit.trace(best_model, example_input)
    out_dir = pkg_dir/'model'
    if (not out_dir.exists()):
        out_dir.mkdir()
    
    # モデルを保存
    date_str = datetime.datetime.now().strftime("%Y%m%d_%H%M")
    save_path = out_dir/f'NN_{label_to_learn}_{date_str}.pt'
    scripted_model.save(save_path)
    print(f"Model saved to {save_path}")
