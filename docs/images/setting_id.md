# 1. EPOS Studioのインストール
[公式ページ](https://www.maxongroup.co.jp/maxon/view/content/epos-detailsite)からダウンロード後，インストーラを実行．ファイルサイズがかなり重い(約2GB)ので余裕がある時にインストールすることを推奨．
# 2. 初期設定
## 2.1. プロジェクトの作成
起動時，新規プロジェクトを作成するウィザードが自動で出現する．そこで，"EPOS4 Project"を選択し，任意のプロジェクト名で作成する．
## 2.2. EPOSとの接続
まず，PCとEPOSをUSBケーブルで物理的に接続する．次に，画面左上のConnect All![connectall](uploads/cb18436dd25b30142841176888f50be7/connectall.png)を押し，ソフト的に接続する．なお，接続を解除する場合は，Connect Allの隣のDisconnect All![disconnectall](uploads/e4aa65364dcd22275eb94c0fe8e431a5/disconnectall.png)を押せば良い．
最後に，左上のWorkspace内のEPOS4 CAN[Node n]を右クリックし[^1]，Scanning Devicesを選択する．立ち上がった画面でスキャンを実行[^2]．
[^1]:nには接続したEPOS固有の番号が表示される
[^2]:r5srの構成なら5つのデバイスが見つかった時点でスキャンを中断して良い
## 2.3. モータの初期設定
Workspace内のノードのうち，設定したい番号のノードを右クリックし，Connectを選択する．[^3]
接続後，右クリック画面内のWizardsより，Startupを選択する．(初回のみメッセージが出るので，チェックをつけて次に進む．)
[ここ](足回りモータリスト)のデータシートを参考に，各モータに適切なパラメータを設定する．
[^3]:2.2節で出てきたConnect All![connectall](uploads/cb18436dd25b30142841176888f50be7/connectall.png)で全て同時に接続してもいいが，処理が重いのか筆者のノートPCだとフリーズが頻発した
# 3. 動作テスト
動かしたいノードを右クリックし，ToolsからProfile Velocity Modeを選択する．新たに設定画面が出現するので，まず左下のenableを押す．次にTarget velocityを指定し，Set velocityを押すと，指定された回転数でモータが回転する．停止する際は，Quick stopで停止する．
# 注釈
----
# 設定ファイル置き場

https://drive.google.com/drive/folders/1la63z8yKuOVlImASu7qKBF9ZNuc-dYnz?usp=sharing