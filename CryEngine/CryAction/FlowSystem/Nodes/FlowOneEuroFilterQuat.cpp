/* Onde Euro Filter quaternion node - for licensing and copyright see license.txt */

#include "StdAfx.h"
#include "Nodes/G2FlowBaseNode.h"
#include "Actor.h"
#include "Filters/OneEuroFilter.h"

class CFlowOneEuroFilterQuat : public CFlowBaseNode<eNCT_Instanced>
{
	OneEuroFilter *oef_w;
	OneEuroFilter *oef_x;
	OneEuroFilter *oef_y;
	OneEuroFilter *oef_z;
	
	Quat previous;
	bool first;

	enum EInputPorts
	{
		ACTIVE,
		IN_Value,
		IN_Value_w,
		
	};

	enum EOutputPorts
	{
		OUT_Value,
		OUT_Value_w,
	};

public:
	bool active;
	
	////////////////////////////////////////////////////
	CFlowOneEuroFilterQuat(SActivationInfo *pActInfo){
		
	}
	////////////////////////////////////////////////////
	virtual ~CFlowOneEuroFilterQuat(void){}
	////////////////////////////////////////////////////
	virtual void Serialize(SActivationInfo *pActInfo, TSerialize ser){}

	////////////////////////////////////////////////////
	virtual void GetConfiguration(SFlowNodeConfig& config)
	{
		 // Define input ports here, in same order as EInputPorts
            static const SInputPortConfig inputs[] =
            {
                InputPortConfig<bool> ( "Activate", _HELP( "Activation" ) ),
                InputPortConfig<Vec3> ( "xyz_value", Vec3( 0, 0, 0 ), _HELP( "XYZ quaternion part to filter" ) ),
                InputPortConfig<float> ( "w_value", 0, _HELP( "W quaternion part to filter" ) ),
                {0},
            };

            // Define output ports here, in same order as EOutputPorts
            static const SOutputPortConfig outputs[] =
            {
                OutputPortConfig<Vec3> ( "FilteredValue_xyz", _HELP( "Filtered XYZ quaternion part" ) ),
                OutputPortConfig<float> ( "FilteredValue_w", _HELP( "Filtered W quaternion part" ) ),
                {0},
            };

            // Fill in configuration
            config.pInputPorts = inputs;
            config.pOutputPorts = outputs;
            config.sDescription = _HELP( "FG node that sets up a CAVE environment" );
            config.SetCategory( EFLN_APPROVED );
	}

	


	////////////////////////////////////////////////////
	virtual void ProcessEvent(EFlowEvent event, SActivationInfo *pActInfo)
	{	

		switch (event)
		{
		case eFE_Initialize:
			{
			first = true;	
				oef_w = new OneEuroFilter ();
				oef_x = new OneEuroFilter ();
				oef_y = new OneEuroFilter ();
				oef_z = new OneEuroFilter ();

			}
			break;
		case eFE_Activate:
			{
				if ( IsPortActive( pActInfo, ACTIVE ) ){
					active = GetPortBool( pActInfo, ACTIVE);
					pActInfo->pGraph->SetRegularlyUpdated(pActInfo->myID,true);
				}

			}
			break;
		
		case eFE_Update:
			{
				if(!active) return;
				
				oef_w->increaseTimeStamp(1.0/oef_w->getFrequence());
				oef_x->increaseTimeStamp(1.0/oef_x->getFrequence());
				oef_y->increaseTimeStamp(1.0/oef_y->getFrequence());
				oef_z->increaseTimeStamp(1.0/oef_z->getFrequence());

				Vec3 valeur =  GetPortVec3(pActInfo, IN_Value);
				Quat q = Quat(GetPortFloat(pActInfo, IN_Value_w),valeur.x,valeur.y,valeur.z);
				Ang3 ang = Ang3(q);
				


				if(first) {
					previous = q;
					first = false;
				}
				
				if((q.v.x * previous.v.x + q.v.y * previous.v.y + q.v.z * previous.v.z + q.w * previous.w)<0) q= -q;
				
				previous = q;
				//AngleAxis a = AngleAxis(q);
				
				valeur = Vec3(q.v.x,q.v.y,q.v.z);
				float w = q.w;

				float filtered_x = (float)oef_x->filter(valeur.x) ;
				float filtered_y = (float)oef_y->filter(valeur.y) ;
				float filtered_z = (float)oef_z->filter(valeur.z) ;
				float filtered_w = (float)oef_w->filter(w) ;

				Quat quat = Quat(filtered_w,filtered_x,filtered_y,filtered_z);
				
				quat.Normalize();

				



				Vec3 filtered = Vec3(quat.v.x,quat.v.y,quat.v.z);
				
				ActivateOutput(pActInfo, OUT_Value,filtered);
				ActivateOutput(pActInfo, OUT_Value_w,quat.w);
			}
		}
	}

	////////////////////////////////////////////////////
	virtual void GetMemoryStatistics(ICrySizer *s)
	{
		s->Add(*this);
	}

	////////////////////////////////////////////////////
	virtual IFlowNodePtr Clone(SActivationInfo *pActInfo)
	{
		return new CFlowOneEuroFilterQuat(pActInfo);
	}

	virtual void GetMemoryUsage(ICrySizer * s) const
	{
		s->Add(*this);
	}


};


REGISTER_FLOW_NODE("CryVR:Filters:OneEuroFilterQuaternion", CFlowOneEuroFilterQuat);
