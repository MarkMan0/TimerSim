#include "CppUnitTest.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

#include "../TimerSim/Timer.h"
#include "../TimerSim/Timer.cpp"

namespace TimerSimTests {

	int val = 0;
	void incVal() {
		val++;
	}



	TEST_CLASS(TimerTests) {


		TEST_METHOD(TestCallback) {

			val = 0;
			Timer timer(incVal);
			
			timer.setPrescale(2);
			timer.setThreshold(5);

			timer.tick();
			Assert::AreEqual(0, val, L"Callback called on first tick()");
			
			timer.tick();
			Assert::AreEqual(static_cast<Timer::counter_t>(1), timer.getCounter(), L"Counter not 1 after two ticks");


			timer.reset();

			for (int i = 0; i < 9; ++i) {
				timer.tick();
				Assert::AreEqual(static_cast<Timer::counter_t>((i + 1) / 2), timer.getCounter(), L"Counter does not match expected");
			}
			Assert::AreEqual(0, val, L"val not 0");
			timer.tick();
			Assert::AreEqual(1, val, L"val not 1");
			Assert::AreEqual(static_cast<Timer::counter_t>(0), timer.getCounter(), L"Counter does not match expected");
		}
	};

};