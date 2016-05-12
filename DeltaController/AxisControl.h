#pragma once

#include <nyceapi.h>
#include <sacapi.h>

//终止控制轴
NYCE_STATUS TermAxis(const uint32_t &axesNum, const SAC_AXIS* const axId)
{
	SAC_STATE sacState;
	SAC_SPG_STATE sacSpgState;
	NYCE_STATUS nyceStatus(NYCE_OK);
	for (uint32_t ax = 0; ax < axesNum; ax++ )
	{
		//读取轴状态
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacReadState(axId[ ax ], &sacState, &sacSpgState);
		if(NyceSuccess(nyceStatus) && sacState == SAC_MOVING)
		{
			//如果正在运动，急停
			nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacQuickStop(axId[ ax ]);
			if (NyceSuccess(nyceStatus))
			{
				nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacSynchronize( axId[ ax ], SAC_REQ_MOTION_STOPPED, 10 );
				if (NyceError(nyceStatus))
				{
					//急停同步超时或失败，重置轴
					nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacReset(axId[ ax ]);
					nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacSynchronize( axId[ ax ], SAC_REQ_RESET, 10 );
				}
			}
		}

		//shutdown轴
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacShutdown(axId[ ax ]);
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacSynchronize( axId[ ax ], SAC_REQ_SHUTDOWN, 10 );

		//断开连接
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacDisconnect(axId[ ax ]);
		if(NyceError(nyceStatus)) return nyceStatus;
	}
	return nyceStatus;
}

//初始化轴，适用三洋编码器协议
NYCE_STATUS InitAxisSanyo(const uint32_t &axesNum, SAC_AXIS* const axId, const char *axName[])
{
	SAC_SPG_STATE sacSpgState;
	SAC_STATE sacState;
	SAC_CONFIGURE_AXIS_PARS axisPars;
	double signal = 0;
	NYCE_STATUS nyceStatus(NYCE_OK);

	for (uint32_t ax = 0; ax < axesNum; ax++ )
	{
		//连接轴
		nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacConnect( axName[ ax ], &axId[ ax ] );

		//查询轴状态
		nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacReadState( axId[ ax ], &sacState, &sacSpgState);
		if(NyceSuccess(nyceStatus))
		{
			switch (sacState)
			{
			//其他情况，shutdown轴
			default:
				nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacShutdown( axId[ ax ]);
				nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacSynchronize( axId[ ax ], SAC_REQ_SHUTDOWN, 10 );

			//轴在IDLE状态时，初始化轴
			case SAC_IDLE:
				nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacInitialize( axId[ ax ], SAC_USE_FLASH );
				nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacSynchronize( axId[ ax ], SAC_REQ_INITIALIZE, 10 );
				goto INACTIVE;

			//轴在错误状态时，清除错误
			case SAC_ERROR:
				nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacReset(axId[ ax ]);
				nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacSynchronize(axId[ ax ], SAC_REQ_RESET,10);

			//轴回零，寻相，lock后进入Ready状态
			case SAC_INACTIVE:
INACTIVE:		nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacHome( axId[ ax ] );
				nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacSynchronize( axId[ ax ], SAC_REQ_HOMING_COMPLETED, 10 );
				
				nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacReadVariable(axId[ ax ], SAC_VAR_BLAC_ALIGNED, &signal);
				if (signal == 0)
				{
					nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacGetAxisConfiguration( axId[ ax ], &axisPars );
					if ( NyceSuccess(nyceStatus) && axisPars.motorType == SAC_BRUSHLESS_AC_MOTOR )
					{
						nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacAlignMotor( axId[ ax ] );
						nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacSynchronize( axId[ ax ], SAC_REQ_ALIGN_MOTOR, 10 );
					}
				}

				nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacLock( axId[ ax ] );
				nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacSynchronize( axId[ ax ], SAC_REQ_LOCK, 10 );

				break;
			}

		}

		if(NyceError(nyceStatus))
			return nyceStatus;
	}
	return nyceStatus;
}

//初始化轴，用在中南林大Delta机器人上
NYCE_STATUS InitAxisRexroth(const uint32_t &axesNum, SAC_AXIS* const axId, const char *axName[])
{
	SAC_SPG_STATE sacSpgState;
	SAC_STATE sacState;
	SAC_CONFIGURE_AXIS_PARS axisPars;
	double signal = 0;
	NYCE_STATUS nyceStatus(NYCE_OK);
	for (uint32_t ax = 0; ax < axesNum; ax++ )
	{
		//连接轴
		nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacConnect( axName[ ax ], &axId[ ax ] );

		//读取轴状态
		nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacReadState( axId[ ax ], &sacState, &sacSpgState);
		if(NyceSuccess(nyceStatus))
		{
			switch (sacState)
			{
			//其他情况，shutdown轴
			default:
				nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacShutdown( axId[ ax ]);
				nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacSynchronize( axId[ ax ], SAC_REQ_SHUTDOWN, 10 );

			//IDLE状态，初始化轴
			case SAC_IDLE:
				nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacInitialize( axId[ ax ], SAC_USE_FLASH );
				nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacSynchronize( axId[ ax ], SAC_REQ_INITIALIZE, 10 );
				goto INACTIVE;

			//错误状态，清除错误
			case SAC_ERROR:
				nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacReset(axId[ ax ]);
				nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacSynchronize(axId[ ax ], SAC_REQ_RESET,10);
			
			//寻相，lock，回零后进入Ready状态
			case SAC_INACTIVE:
INACTIVE:		
				nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacReadVariable(axId[ ax ], SAC_VAR_BLAC_ALIGNED, &signal);
				if (signal == 0)
				{
					nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacGetAxisConfiguration( axId[ ax ], &axisPars );
					if ( NyceSuccess(nyceStatus) && axisPars.motorType == SAC_BRUSHLESS_AC_MOTOR )
					{
						nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacAlignMotor( axId[ ax ] );
						nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacSynchronize( axId[ ax ], SAC_REQ_ALIGN_MOTOR, 10 );
					}
				}

				nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacLock( axId[ ax ] );
				nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacSynchronize( axId[ ax ], SAC_REQ_LOCK, 10 );

				nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacHome( axId[ ax ] );
				nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacSynchronize( axId[ ax ], SAC_REQ_HOMING_COMPLETED, 10 );

				break;
			}
		}

		if(NyceError(nyceStatus))
			return nyceStatus;
	}

	return nyceStatus;
}