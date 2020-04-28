# Markdown输入

==高亮==

`==高亮==`

**加粗**

`**加粗**`

*倾斜*

`*倾斜*`

***倾斜加粗***

`***倾斜加粗***`

~~删除线~~

`~~删除线~~`

+ 分级
+ `+ 分级`

1. 计数
2. `1. 计数`

| 表格 |      |
| ---- | ---- |
|      |      |

`|表格| |`

> 引用

`>引用`

**分割线**

-----------

`---------`

```代码框
​```
代码框
​```
```

- [ ] 任务列表

`- [ ]`

![]()

`![]()`

## 数学公式

在`$$`之间输入公式即可得到数学公式

输入`$$`并回车即可得到公式框

示例：`${1\over{1+{9\over4}}}({1\over2}\cos3xe^{2x}+{3\over4}\sin e^{2x})+e`

$${1\over{1+{9\over4}}}({1\over2}\cos3xe^{2x}+{3\over4}\sin e^{2x})+e$$
$$
{1\over{1+{9\over4}}}({1\over2}\cos3xe^{2x}+{3\over4}\sin e^{2x})+e
$$
$\LaTeX$

## 颜色

示例：`$\textcolor{blue}{蓝色}$`

$\textcolor{blue}{蓝色}$

---------------

##希腊字母

|    大写    |    小写    |  英文   |    大写    |    小写    |  英文   |
| :--------: | :--------: | :-----: | :--------: | :--------: | :-----: |
|  $\Alpha$  |  $\alpha$  |  alpha  |     N      |     v      |   nu    |
|  $\Beta$   |  $\beta$   |  beta   |   $\Xi$    |   $\xi$    |   xi    |
|  $\Gamma$  |  $\gamma$  |  gamma  | $\Omicron$ | $\omicron$ | omicron |
|  $\Delta$  |  $\delta$  |  delta  |   $\Pi$    |   $\pi$    |   pi    |
| $\Epsilon$ | $\epsilon$ | epsilon |   $\Rho$   |   $\rho$   |   rho   |
|  $\Zeta$   |  $\zeta$   |  zeta   |  $\Sigma$  |  $\sigma$  |  sigma  |
|   $\Eta$   |   $\eta$   |   eta   |   $\Tau$   |   $\tau$   |   tau   |
|  $\Theta$  |  $\theta$  |  theta  | $\Upsilon$ | $\upsilon$ | upsilon |
|  $\Iota$   |  $\iota$   |  iota   |   $\Phi$   |   $\phi$   |   phi   |
|  $\Kappa$  |  $\kappa$  |  kappa  |   $\Chi$   |   $\chi$   |   chi   |
| $\Lambda$  | $\lambda$  | lambda  |   $\Psi$   |   $\psi$   |   psi   |
|   $\Mu$    |   $\mu$    |   mu    |  $\Omega$  |  $\omega$  |  omega  |

## 上下标

^ 表示上标， _ 表示下标，如果上标或下标内容多于一个字符，则使用 {} 括起来

示例： `x^{y^z} = (1+e^x)^{-2xy^w}`

$x^{y^z} = (1+e^x)^{-2xy^w}$

示例：`\sum_{i=0}^5 = \sum\limits_{i=0}^5​`

$\sum_{i=0}^5 = \sum\limits_{i=0}^5$

## 分数

使用`\frac`*{分子}{分母}*，或者使用 *分子 \over 分母*

示例： `\frac{1}{2x+1} , {{1} \over {2x+1}}`

$\frac{1}{2x+1} , {{1} \over {2x+1}}$

加一个d更大

示例：`\dfrac{b}{a}`

$\dfrac{b}{a}$

## 开方

使用 *\sqrt[n]{a}*

示例： `\sqrt[3]{9}, \sqrt{16}`

$\sqrt [n]{a}$

## 向量

使用 *`\vec`{a}*

示例： `\vec a \cdot \vec b = 0`

$\vec a \cdot \vec b = 0$

## 积分

示例： `\int_0^1 x^2dx`

$\int_0^1 x^2dx$

## 极限

示例： `\lim\limits_{n\rightarrow+\infty}\frac{1}{n(n+1)}`

$\lim\limits_{n\rightarrow+\infty}\frac{1}{n(n+1)}$

## 累加/累乘

示例： `\sum_1^n\frac{1}{x^2}, \prod_{i=0}^n{1 \over {x^2}}`

$ `\sum_1^n\frac{1}{x^2}, \prod_{i=0}^n{1 \over {x^2}}`$

## 需要转义的字符

示例： `\# \$ \%\&\_\{\}`

$ \# \$ \%\&\_\{\}$

## 括号

示例：`\overbrace{a+\underbrace{b+c}_{1.0}+d}^{2.0}`

$\overbrace{a+\underbrace{b+c}_{1.0}+d}^{2.0}$

## 空格

示例：`A \quad B`

$A\quad B$

## 大括号

示例：`\left \{ \begin {array}{r**} a+b=3\\a-b=1 \end{array} \right.`

$\left \{ \begin {array}{r**} a+b=3\\a-b=1 \end{array} \right.$

## 符号

|      符号       |      写法      |       符号       |      写法       |       符号       |      写法       |       符号        |     写法      |
| :-------------: | :------------: | :--------------: | :-------------: | :--------------: | :-------------: | :---------------: | :-----------: |
|      $\pm$      |      `pm`      |     $\times$     |     `times`     |      $\div$      |      `div`      |      $\mid$       |     `mid`     |
|     $\cdot$     |     `cdot`     |     $\circ$      |     `circ`      |      $\ast$      |      `ast`      |    $\bigodot$     |   `bigodot`   |
|  $\bigotimes$   |  `bigotimes`   |      $\leq$      |      `leq`      |      $\geq$      |      `geq`      |      $\neq$       |     `neq`     |
|    $\approx$    |    `approx`    |     $\equiv$     |     `equiv`     |      $\sum$      |      `sum`      |      $\prod$      |    `prod`     |
|    $\coprod$    |    `coprod`    |   $\emptyset$    |   `emptyset`    |      $\in$       |      `in`       |     $\notin$      |    `notin`    |
|    $\subset$    |    `subset`    |    $\supset$     |    `supset`     |   $\subseteq$    |   `subseteq`    |    $\supseteq$    |  `supseteq`   |
|    $\bigcap$    |    `bigcap`    |    $\bigcup$     |    `bitcup`     |    $\bigvee$     |    `bigvee`     |    $\bigwedge$    |  `bigwedge`   |
|   $\biguplus$   |   `biguplus`   |   $\bigsqcup$    |   `bitsqcup`    |      $\bot$      |      `bot`      | $\angle 30^\circ$ |  `angle 30`   |
|     $\sin$      |     `sin`      |      $\cos$      |      `cos`      |      $\tan$      |      `tan`      |      $\cot$       |     `cot`     |
|     $\sec$      |     `sec`      |      $\log$      |      `log`      |      $\lg$       |      `lg`       |       $\ln$       |     `ln`      |
|    $\prime$     |     prime      |      $\int$      |      `int`      |    ${\iint}$     |     `iint`      |    ${\iiint}$     |    `iiint`    |
|     $\oint$     |     `oint`     |      $\lim$      |      `lim`      |     $\infty$     |     `infty`     |     $\nabla$      |    `nabla`    |
|   $\because$    |   `because`    |   $\therefore$   |   `therefore`   |    $\forall$     |    `forall`     |     $\exists$     |   `exists`    |
|   $\uparrow$    |   `uparrow`    |   $\downarrow$   |   `downarrow`   |   $\leftarrow$   |   `leftarrow`   |   $\rightarrow$   | `rightarrow`  |
|   $\Uparrow$    |   `Uparrow`    | $\longleftarrow$ | `longleftarrow` | $\Longleftarrow$ | `Longleftarrow` |  $\overline{a}$   | `overline{a}` |
| $\underline{a}$ | `underline{a}` |    $\hat{y}$     |    `hat{y}`     |   $\check{y}$    |   `check{y}`    |    $\breve{y}$    |  `breve{y}`   |
|    $\langle$    |    `langle`    |    $\rangle$     |    `rangle`     |     $\cdots$     |     `cdots`     |   $\triangleq$    |  `traingleq`  |
|    $\vec{E}$    |    `vec{E}`    |   $\mathrm{d}$   |   `mathrm{d}`   |                  |                 |                   |               |
|                 |                |                  |                 |                  |                 |                   |               |

$$
\overrightarrow{E(\vec{r})}=\int_{vol}\dfrac{\rho_v(\vec{r}')\mathrm{d}v'}{4\pi \epsilon_0 \vert \vec{r}-\vec{r}'\vert^2}\cdot\dfrac{\vec{r}-\vec{r}'}{\vert\vec{r}-\vec{r}'\vert}
$$

