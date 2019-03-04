#include <lib/hsm/include/hsm.h>

using namespace hsm;
namespace hsm {
class Commander
{
public:
	Commander();
	void Update();

	// Public to simplify sample
	bool mMove;
	bool mAttack;

private:
	friend struct CommanderStates;
	StateMachine mStateMachine;
};

struct CommanderStates
{
	struct BaseState : StateWithOwner<Commander>
	{
	};

	struct Alive : BaseState
	{
		DEFINE_HSM_STATE(Alive);
		virtual Transition GetTransition()
		{
			return InnerEntryTransition<Locomotion>();
		}
	};

	struct Locomotion : BaseState
	{
		DEFINE_HSM_STATE(Locomotion);		
		virtual Transition GetTransition()
		{
			if (Owner().mAttack)
			{
				// Start attack sequence with combo index 0
				return SiblingTransition<Attack>(0);
			}

			return InnerEntryTransition<Stand>();
		}
	};

	struct Stand : BaseState
	{
		DEFINE_HSM_STATE(Stand);
		virtual Transition GetTransition()
		{
			if (Owner().mMove)
				return SiblingTransition<Move>();

			return NoTransition();
		}
	};

	struct Move : BaseState
	{
		DEFINE_HSM_STATE(Move);
		virtual Transition GetTransition()
		{
			if (!Owner().mMove)
				return SiblingTransition<Stand>();

			return NoTransition();
		}
	};

	struct Attack : BaseState
	{
		DEFINE_HSM_STATE(Attack);
		virtual void OnEnter(int comboIndex)
		{
			Owner().mAttack = false;

			mComboIndex = comboIndex;

			//Owner().mAnimComponent.PlayAnim(AttackAnim[mComboIndex]);
		}

		virtual Transition GetTransition()
		{
			// Check if player can chain next attack
			if (Owner().mAttack
				&& mComboIndex < 2)
			{
				// Restart state with next combo index
				return SiblingTransition<Attack>(mComboIndex + 1);
			}

			// if (Owner().mAnimComponent.IsFinished())
			// 	return SiblingTransition<Locomotion>();

			return NoTransition();
		}

		virtual void Update()
		{
			printf(">>> Attacking: %d\n", mComboIndex);
		}

		int mComboIndex;
	};
};

Commander::Commander()
	: mMove(false)
	, mAttack(false)
{
	mStateMachine.Initialize<CommanderStates::Alive>(this);
	mStateMachine.SetDebugInfo("TestHsm", TraceLevel::Diagnostic);
}

void Commander::Update()
{
	// Update state machine
	mStateMachine.ProcessStateTransitions();
	mStateMachine.UpdateStates();
}
}