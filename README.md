# LoogPhysics简介
这是一个参考自Magica Cloth 的骨骼布料模拟插件，适用于偏二次元的角色飘带模拟效果。目前已经完成了基础版本。

# 动机
1. Unity有非常好用的骨骼布料模拟插件Magica Cloth。但是Unreal仅有Kawaii/SPCR两个开源的骨骼模拟插件，在应对基础模拟需求时这两款插件确实已经可以应付了。但涉及更好表现时，可以发现这两款插件的表现效果与MagicaCloth还是有很大的差距，并且这两款插件的模拟性能非常堪忧，如需用到项目还需要进行性能优化。从已上线的游戏来看，使用骨骼模拟布料的游戏都是不开源的，如叠纸游戏的无限暖暖，FF7Remake/FF7Rebirth等。
2. 虽然unreal自带的Chaos Cloth非常强大，但一旦放到移动端或者低配设备上就会有非常大的性能问题。并且ChaosCloth这种布料模拟方式难以实现二次元风格的布料表现。
3. 刚好本人对也有涉及相关工作，因而也当作一次学习和锻炼

# 版权信息
1. 项目内使用了无限暖暖(Infinity Nikki https://infinitynikki.nuanpaper.com/home )的资产作为验证资产。如有涉及侵权行为请联系我删除
2. 代码实现参考自MagicaClothV2(https://magicasoft.jp/)(https://assetstore.unity.com/packages/tools/physics/magica-cloth-2-242307)作为学习和参考
3. 本项目代码为开源代码，请随意使用，且无需注明出处。

# 效果展示

# GetStart
## Case 1 仅仅想要游玩一下
直接拉取整个项目，并使用最新的Unreal5.7.1打开项目，然后点击play即可体验。

## Case 2 想要在自己的项目中使用该插件
1. 需要将本项目Plugins目录内的整个LoogPhysics文件夹copy到你自己项目的Plugins(注意：是项目的Plugins目录而不是引擎的Plugins)
2. 如果你的引擎刚好是我编译完成的引擎版本(当前为5.7.1)那么你就可以直接运行。否则需要自行编译。（编译需要完整的UnrealEngine C++编译环境）。

## 教学视频和文档
TODO

# 作者
https://github.com/LoogLong

# 打赏
我一个搬运工，就不需要打赏啦
