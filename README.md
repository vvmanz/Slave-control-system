# Slave control system
 Slave control system
Система подчиненного регулирования для STM32F103C8T6:
а) в файле main.c располагается следующий функционал:
- Регулятор с последовательной коррекцией для каждого привода (функция pid_1, pid_2)
- Считывание данных с АЦП через DMA (if adcFlag==1)
- Фильтрация данных для АЦП (функции Median_ADC_CS1,Median_Cur_CS1,simpleKalman1)
- Перерасчет значений из АЦП в силу тока и напряжение(currentSearch)

б) в файле stm32f1xx_it.c следующее:
- Подсчет тиков с энкодера (TIM1_UP_IRQHandler,TIM2_IRQHandler)
- Подсчет скорости и угла поворота(TIM3_IRQHandler)
в) в остальных .c файлах автоматически сгенерированные настройки для АЦП, DMA, таймеров, GPIO и т.д.
г) в файле main.h просто дефайны и константы (опроные напряжения, среднее значение АЦП, предделитель для датчика тока,
уровень квантования и т.д.)