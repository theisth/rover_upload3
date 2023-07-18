# Drive-System-Stm32F4 all notes

STM32 çalışma frekansı 84 Mhz
Pwm sinyali üzeten timer bus 84 Mhz'de çalışmakta
Sürücü çalışma frekansı 16 Hz (PWM Input Rate (Period) = 62,5 ms) --> (bknz. Sayfa 3) (https://store.ctr-electronics.com/content/user-manual/Victor%20SPX%20User's%20Guide.pdf)
----------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------
-------------------------------Formüller------------------------------------------------
## Period = (Timer_Tick_Freq/PWM_Freq) - 1
## PWM_Freq = Timer_Tick_Freq/(Period + 1)
## Timer_Tick_Freq = Timer_CLK/(Prescaler + 1)
----------------------------------------------------------------------------------------
Timer_CLK = 84 MHz (Timer Bus Frequency) (STM32 maxsimum çalışabileceği frekansta ayarlandı)
Sürücümüz 2,9ms-100ms aralığında istediğimiz periyotta çalışabilir. Biz 1 PWM periyodunu ne kadar uzun tutabilirsek belirlediğimiz duty cycle o kadar hassas olacaktır. (Örnek ile göstereceğim)
Elimizde Victor Spx sürücüsü Servo mantığı ile çalışmaktadır. 1-2 ms değer aralığı almaktadır. (1 ms tam geri 2ms tam ileri gibi) 1,5 ms değer verdiğimizde sürücü tam orta noktada olacağı için (1-2 ms değer aralığının tam ortası) motorun hareket etmesin beklemeyiz.

Örnek
-Sürücü 100 ms de çalışsın diyelim frekansı 10 Hz olur (1sn / 100 ms)
PWM_Freq = 10 olur
-Timer_CLK = 84 MHz olarak stm32' de ayarlıydı.
-Prescaler değerini öyle bir seçelim ki Timer_Tick_Freq hesaplaması rahat ve düzgün olsun.
Timer_Tick_Freq = 84 Mhz / 84 = 1 Mhz dersek yani (Prescaler + 1) değerini 84 seçersek prescaler 83 olur. (Prescaler = 83)
-PWM_Freq = 10 Hz, Timer_Tick_Freq = 1 Mhz (Period + 1) = 100000 olur Yani periyot = 99999 olur.
- %100 duty cycle da çalıştırmak istesek 99999 vermemiz gerekiyor demek bu. Fakat biz 1-2 ms değer aralığında sürücüyü çalıştırmak istiyoruz.
1-2 ms değerleri kaçlık duty cycle denk geliyor onu hesaplamalıyız.
100 ms bizim sürücümüzün çalışma periyoduydu. 1 ms de çalışmasını istersek %1 lik duty cycle üretmemiz gerekiyordu ((1ms/100ms)*%100) 2ms değeri için ise %2'lik duty cycle olması gerekiyor.
-%100 duty cycle 99999 periyoduna denk geliyordu %1 99,999 a denk gelir yani 100 değerine.
%2 duty cycle ise 200 değerine tekabül eder. Biz yazdığımız kodda 100 değerini verirsek motor tam ileri 200 değerini verirsek motor tam geri dönecektir. (yönlere takılmayın farklı tarafa tam güçte döneceği için rasgele ileri geri dedim.) 
Sonuc olarak 
Timer_CLK = 84 Mhz
Timer_Tick_Freq = 1 Mhz
PWM_Freq = 10 Hz
Period = 99999
Prescaler = 83 oldu
Bunları konfigre ederek işe başlıyoruz.
Bu konfigrasyonlardan sonra max ileri ve max geri periyot değerlerimizi de hesapladığımız içim biliyoruz (100 ve 200) Tam 150 periyoty değerinde de motorlar dönmeyecektir.

Örnek tamamlandı..

Sürücüyü 100 ms de çalıştırmak sürücünün %100'ünü kullanmaya tekabül ediyor. Fakat biz sürücünün %100' ünü kullanmak istemiyoruz çünkü ısınıp mosfetleri yanıyor soğutucusu olmadığı için bu sebeple 62.5 ms de çalışsın olarak seçtik. Bu da PWM_Freq = 16 Hz e takbül ediyor. 
Mevcut kodda prescaler = 83 , Period = 62499 olarak ayarlıdır. Yine yukarıdaki örnek gibi hesaplama yaparsak %1 duty cycle için periyodumuz 1000 değerini %2 duty cycle için  periyodumuz 2000 değerini alır. Tam 1500 ise %1,5 duty cycle tekabül eder.
NOT: 1500 değerinde tekerlerin bir miktar hareket ettiği gözlemlendi. Bu sebeple 1525 değeri %1,5 duty cycle tekabül ediyor diye ayarlandı. Tekerlerin bu şekilde hareket etmesinin nedeni STM32 kartında internal osilatör kullanılıyor bu da tam doğru bir çevrim sağlamadığı için düzgün PWM_Freq değerine ulaşamıyoruz. Bizim ayarladığımız 16 Hz PWM_Freq değeri 16,20 Hz olarak osiloskopta gözlemlendi. 
