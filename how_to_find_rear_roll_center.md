# rear roll center
## 建立 Roll Moment 平衡
定義後軸 Roll Stiffness 比
$$
\lambda_K=\frac{K_{\phi,r}}{K_{\phi,f}+K_{\phi,r}}
$$

## 側向力
$$
F_{y,f}=ma_y\frac{W_f}{W_f+W_r}
$$

$$
F_{y,r}=ma_y\frac{W_r}{W_f+W_r}
$$

$$
M_\phi=
F_{y,f}(h_{CG}-h_{RC,f})
+
F_{y,r}(h_{CG}-h_{RC,r})
$$

前軸載重轉移
$$
\boxed{
\Delta F_{z,f}=
\frac{F_{y,f}h_{RC,f}+(1-\lambda_K)M_\phi}{t_f}
}
$$

後軸載重轉移
$$
\boxed{
\Delta F_{z,r}=
\frac{F_{y,r}h_{RC,r}+\lambda_K M_\phi}{t_r}
}
$$
## 求解
設定前後重量轉移相同
$$
\boxed{\Delta F_{z,f}=\Delta F_{z,r}}
$$

$$
\frac{F_{y,f}h_{RC,f}+(1-\lambda_K)M_\phi}{t_f}
=
\frac{F_{y,r}h_{RC,r}+\lambda_K M_\phi}{t_r}
$$

$$
A=\frac{F_{y,f}}{t_f},\qquad
B=\frac{F_{y,r}}{t_r}
$$
$$
C=\frac{1-\lambda_K}{t_f}-\frac{\lambda_K}{t_r}
$$

$$
Bh_{RC,r}-Ah_{RC,f}
=
C\left[(F_{y,f}+F_{y,r})h_{CG}
-F_{y,f}h_{RC,f}-F_{y,r}h_{RC,r}\right]
$$

$$
\boxed{
h_{RC,r}=
\frac{
Ah_{RC,f}
+
C\left[(F_{y,f}+F_{y,r})h_{CG}-F_{y,f}h_{RC,f}\right]
}{
B+CF_{y,r}
}
}
$$
