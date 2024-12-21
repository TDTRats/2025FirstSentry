#ifndef __STATE_TASK_H
#define __STATE_TASK_H

/*�����࣬ÿ�θ�ֵ��ʱ�����Ƿ�仯�����仯����ô���Ĳ�����

	State_class<int, MyClass> aaa(&instance, &MyClass::onStateChange);
	onState�������޲�����Ҳ����ʱ�в��������в���a��b�������ʱaĬ�ϴ���nowstate��bĬ�ϴ���newstate
	State<int> aaa(&onStateChange); ͬ��
*/

#include "board.h" 
#include "task_virtual.h"

template <typename T, typename ClassType>     //������ں�����state��
class State_class {
public:
    using CallbackWithParams = void (ClassType::*)(T, T);
    using CallbackWithoutParams = void (ClassType::*)();

    // ���캯��֧���в������޲����ص�
    State_class(ClassType* instance, CallbackWithParams callbackWithParams)
        : nowState(T()), instance(instance), stateChangeCallback(callbackWithParams), no_stateChangeCallback(nullptr) {}

    State_class(ClassType* instance, CallbackWithoutParams callbackWithoutParams)
        : nowState(T()), instance(instance), stateChangeCallback(nullptr), no_stateChangeCallback(callbackWithoutParams) {}

    State_class()
        : nowState(T()), instance(nullptr), stateChangeCallback(nullptr), no_stateChangeCallback(nullptr) {}

    State_class& operator=(const T& newState) {
        if (nowState != newState) {
            if (no_stateChangeCallback && instance) {
                (instance->*no_stateChangeCallback)();
            } else if (stateChangeCallback && instance) {
                (instance->*stateChangeCallback)(nowState, newState); // ���ô������Ļص�����
            }
						nowState = newState; // ����״̬
        }
        return *this;
    }

    // ǰ�õ���������
    State_class& operator++() {
        *this = nowState + 1; // ʹ�ø�ֵ������
        return *this;
    }

    State_class operator++(int) {
        State_class temp = *this;
        *this = nowState + 1; // ʹ�ø�ֵ������
        return temp;
    }

    State_class& operator+=(const T& value) {
        *this = nowState + value; // ʹ�ø�ֵ������������״̬�������ص�
        return *this;
    }

    // ǰ�õݼ�������
    State_class& operator--() {
        *this = nowState - 1;
        return *this;
    }

    // ���õݼ�������
    State_class operator--(int) {
        State_class temp = *this;
        *this = nowState - 1;
        return temp;
    }

    // ������ֵ������
    State_class& operator-=(const T& value) {
        *this = nowState - value;
        return *this;
    }

    // ������ֵ������
    State_class& operator/=(const T& value) {
        *this = nowState / value;
        return *this;
    }

    // �˷���ֵ������
    State_class& operator*=(const T& value) {
        *this = nowState * value;
        return *this;
    }

    // ת���������������ʽת��Ϊ T
    operator T&() {
        return nowState;
    }

    // �����Ҫ�������ʣ��������� const ����
    operator const T&() const {
        return nowState;
    }

private:
    T nowState; // ��ǰ״̬
    ClassType* instance; // ���ʵ��
    CallbackWithParams stateChangeCallback; // ��������״̬�仯�ص�����
    CallbackWithoutParams no_stateChangeCallback; // �޲�����״̬�仯�ص�����
};



template <typename T>              //���ȫ�ֻ���static������state��
class State {
public:
		
    using CallbackWithParams = void (*)(T, T);;
    using CallbackWithoutParams = void (*)();;

    // ���캯��֧���в������޲����ص�
    State(CallbackWithParams callbackWithParams) 
        : nowState(T()), stateChangeCallback(callbackWithParams), no_stateChangeCallback(nullptr) {}

    State(CallbackWithoutParams callbackWithoutParams) 
        : nowState(T()), stateChangeCallback(nullptr), no_stateChangeCallback(callbackWithoutParams) {}


		// ���캯��֧���в������޲����ص�
    State() 
        : nowState(T()), stateChangeCallback(nullptr), no_stateChangeCallback(nullptr) {}
				
    State& operator=(const T& newState) {
        if (nowState != newState) {
					
						nowState = newState; // ����״̬
						if(no_stateChangeCallback)  no_stateChangeCallback();
            else if(stateChangeCallback) {stateChangeCallback(nowState, newState);} // ���ô������Ļص����� 
						nowState = newState; // ����״̬
        }
        return *this;
    }
		
		// ǰ�õ���������
		State& operator++() {
				*this = nowState + 1; // ʹ�ø�ֵ������
				return *this;
		}
		
		State operator++(int) {
				State temp = *this;
				*this = nowState + 1; // ʹ�ø�ֵ������
				return temp;
		}
		
		State& operator+=(const T& value) {
    *this = nowState + value; // ʹ�ø�ֵ������������״̬�������ص�
    return *this;
		}
				
				// ǰ�õݼ�������
		State& operator--() {
				*this = nowState - 1;
				return *this;
		}

		// ���õݼ�������
		State operator--(int) {
				State temp = *this;
				*this = nowState - 1;
				return temp;
		}

		// ������ֵ������
		State& operator-=(const T& value) {
				*this = nowState - value;
				return *this;
		}

		// ������ֵ������
		State& operator/=(const T& value) {
				*this = nowState / value;
				return *this;
		}

		// �˷���ֵ������
		State& operator*=(const T& value) {
				*this = nowState * value;
				return *this;
		}
		
    // ת���������������ʽת��Ϊ T
    operator T&() {
        return nowState;
    }

    // �����Ҫ�������ʣ��������� const ����
    operator const T&() const {
        return nowState;
    }

private:
    T nowState; // ��ǰ״̬
    CallbackWithParams stateChangeCallback; // ��������״̬�仯�ص�����
    CallbackWithoutParams no_stateChangeCallback; // �޲�����״̬�仯�ص�����
};




class stateTask:public VirtualTask       //״̬����ʵ��
{
public:
	stateTask();
	void run() override;
	void init() override;
};


#endif