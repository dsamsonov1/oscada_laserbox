
//!!! The module name, the file name and the module's license. Change for your need.
//OpenSCADA module DAQ.laserbox file: module.cpp
/***************************************************************************
 *   Copyright (C) 2012 by MyName MyFamily, <my@email.org>                 *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; version 2 of the License.               *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

//!!! System's includings. Add need for your module includings.
#include <unistd.h>
#include <csignal>
#include <fcntl.h>

//!!! OpenSCADA module's API includings. Add need for your module includings.
#include <terror.h>
#include <tsys.h>
#include <tmess.h>
#include <ttypeparam.h>
#include <tdaqs.h>

//!!! Self your module's includings. Add need for your module includings.
#include "module.h"

//!!! Module's meta-information. Change for your module.
//*************************************************
//* Module info!                                  *
#define MOD_ID		"Laserbox"
#define MOD_NAME	_("DAQ laserbox")
#define MOD_TYPE	SDAQ_ID
#define VER_TYPE	SDAQ_VER
#define MOD_VER		"0.0.1"
#define AUTHORS		_("Dmitry Samsonov")
#define DESCRIPTION	_("Laserbox DAQ module.")
#define LICENSE		"Fuck you!"
//*************************************************

EpiLaser ModTmpl::TMdContr::las;  // <---- define static here
int64_t ModTmpl::TMdContr::acqTimer;


ModTmpl::TTpContr *ModTmpl::mod;	//Pointer for direct access to the module

//!!! Required section for binding OpenSCADA core to this module. It provides information and create module root object.
//!!! Do not remove this section!
extern "C"
{
#ifdef MOD_INCL
    TModule::SAt daq_Tmpl_module( int nMod )
#else
    TModule::SAt module( int nMod )
#endif
    {
	if(nMod == 0)	return TModule::SAt(MOD_ID, MOD_TYPE, VER_TYPE);
	return TModule::SAt("");
    }

#ifdef MOD_INCL
    TModule *daq_Tmpl_attach( const TModule::SAt &AtMod, const string &source )
#else
    TModule *attach( const TModule::SAt &AtMod, const string &source )
#endif
    {
	if(AtMod == TModule::SAt(MOD_ID,MOD_TYPE,VER_TYPE))
	    return new ModTmpl::TTpContr(source);
	return NULL;
    }
}

//!!! Include for default enter to your module namespace.
using namespace ModTmpl;

//*************************************************
//* TTpContr                                      *
//*************************************************
//!!! Constructor for Root module object.
TTpContr::TTpContr( string name ) : TTypeDAQ(MOD_ID)
{
    //!!! Init shortcut to module root object. Don't change it!
    mod = this;

    //!!! Load module meta-information to root object. Don't change it!
    modInfoMainSet(MOD_NAME, MOD_TYPE, MOD_VER, AUTHORS, DESCRIPTION, LICENSE, name);
}

//!!! Destructor for Root module object.
TTpContr::~TTpContr( )
{

}

//!!! Module's comandline options for print help function.
string TTpContr::optDescr( )
{
    return TSYS::strMess(_(
	"======================= Module <%s:%s> options =======================\n"
	"---- Parameters of the module section '%s' of the configuration file ----\n\n"),
	MOD_TYPE,MOD_ID,nodePath().c_str());
}

/*
//!!! Processing virtual function for load Root module to DB
void TTpContr::load_( )
{
    //Load parameters from command line
}

//!!! Processing virtual function for save Root module to DB
void TTpContr::save_( )
{

}
*/

//!!! Post-enable processing virtual function
void TTpContr::postEnable( int flag )
{
    TTypeDAQ::postEnable(flag);

    //Controler's bd structure
    fldAdd(new TFld("PRM_BD",_("Parameters table"),TFld::String,TFld::NoFlag,"30",""));
    fldAdd(new TFld("SCHEDULE",_("Acquisition schedule"),TFld::String,TFld::NoFlag,"100","1"));
    fldAdd(new TFld("PRIOR",_("Priority of the acquisition task"),TFld::Integer,TFld::NoFlag,"2","0","-1;99"));

    //!!! Append here your's PLC specific configuration field stored into DB
    fldAdd(new TFld("SERIALID",_("Laserbox serial ID"),TFld::String,TFld::Selected,"16","<default>"));

    //Parameter types and it's bd structure form
    int tPrm = tpParmAdd("std", "PRM_BD", _("Standard")/*, true*/);	//!!! Set "true" here for hierarchically parameters
    tpPrmAt(tPrm).fldAdd(new TFld("OID_LS",_("OID list (next line separated)"),TFld::String,TFld::FullText|TCfg::NoVal,"100",""));
    //!!! Append here your's source object (parameters) types and it's specific configuration field stored into DB
}

//!!! Processing virtual functions for self object-controller creation.
TController *TTpContr::ContrAttach( const string &name, const string &daq_db )
{
    return new TMdContr(name, daq_db, this);
}

//*************************************************
//* TMdContr                                      *
//*************************************************
//!!! Constructor for DAQ-subsystem controller object.
TMdContr::TMdContr( string name_c, const string &daq_db, ::TElem *cfgelem ) : TController(name_c,daq_db,cfgelem),
    prcSt(false), callSt(false), tmGath(0), mSched(cfg("SCHEDULE")), mPrior(cfg("PRIOR")), mPer(1e9),  pEl("w_attr")
{
    cfg("PRM_BD").setS("Laserbox_"+name_c);
}

//!!! Destructor for DAQ-subsystem controller object.
TMdContr::~TMdContr( )
{
    if(startStat()) stop();
}

//!!! Status processing function for DAQ-controllers
string TMdContr::getStatus( )
{
    string rez = TController::getStatus();
    if(startStat() && !redntUse()) {
	//if(!prcSt)	val += TSYS::strMess(_("Task terminated! "));
	if(callSt)	rez += TSYS::strMess(_("Acquisition. "));
	if(period())	rez += TSYS::strMess(_("Acquisition with the period: %s. "), tm2s(1e-9*period()).c_str());
	else rez += TSYS::strMess(_("Next acquisition by the cron '%s'. "), atm2s(TSYS::cron(cron()),"%d-%m-%Y %R").c_str());
	rez += TSYS::strMess(_("Spent time: %s."), tm2s(1e-6*tmGath).c_str());
    }
    return rez;
}

//!!! Processing virtual functions for self object-parameter creation.
TParamContr *TMdContr::ParamAttach( const string &name, int type )
{
    return new TMdPrm(name, &owner().tpPrmAt(type));
}

//!!! Processing virtual functions for start DAQ-controller
void TMdContr::start_( )
{
    if (!startStat()) {

        las.handleStart();

        acqTimer = TSYS::curTime();

        //Start the gathering data task
        SYS->taskCreate(nodePath('.', true), mPrior, TMdContr::Task, this);
    }
}

//!!! Processing virtual functions for stop DAQ-controller
void TMdContr::stop_( )
{
    if (startStat()) {
        las.handleStop();
        las.handleClear();

        //Stop the request and calc data task
        SYS->taskDestroy(nodePath('.', true));
    }
}

void TMdContr::enable_() {

    if (!enableStat()) {
        TController::enable_();
        las.initModule(serialID());
    }

}

void TMdContr::disable_() {

    if (enableStat()) {
        TController::disable_();
        las.destroyModule();
    }
}

//!!! Parameters register function, on time it enable, for fast processing into background task.
void TMdContr::prmEn( TMdPrm *prm, bool val )
{
    unsigned iPrm;

    //MtxAlloc res(enRes.mtx(), true);
    for(iPrm = 0; iPrm < pHD.size(); iPrm++)
	if(&pHD[iPrm].at() == prm) break;

    if(val && iPrm >= pHD.size())	pHD.push_back(prm);
    if(!val && iPrm < pHD.size())	pHD.erase(pHD.begin()+iPrm);
}

//!!! Background task's function for periodic data acquisition.
void *TMdContr::Task( void *icntr )
{
    TMdContr &cntr = *(TMdContr *)icntr;

    cntr.prcSt = true;

    while(!TSYS::taskEndRun()) {
	int64_t t_cnt = TSYS::curTime();

	//Update controller's data
	//!!! Your code for gather data
	las.cyclicFunc();

	std::vector<Point> points;

	las.handleGetData(points);

	//cntr.enRes.lock();
	cntr.callSt = true;
	for(unsigned i_p = 0; i_p < cntr.pHD.size() && !cntr.redntUse(); i_p++)
	    try {
		//!!! Process parameter code
//		    int xxx = cntr.pHD[i_p].at().vlAt("val_red_x1").at().getI();
            for (Point curr : points) {
//                acqTimer += 3333;
                acqTimer += 3500;
                cntr.pHD[i_p].at().vlAt("val_red_x1").at().setI(curr.red[0], acqTimer, true);
                cntr.pHD[i_p].at().vlAt("val_red_x10").at().setI(curr.red[1], acqTimer, true);
                cntr.pHD[i_p].at().vlAt("val_red_x200").at().setI(curr.red[2], acqTimer, true);
                cntr.pHD[i_p].at().vlAt("val_red_ref").at().setI(curr.red[3], acqTimer, true);
                cntr.pHD[i_p].at().vlAt("val_blue_x1").at().setI(curr.blue[0], acqTimer, true);
                cntr.pHD[i_p].at().vlAt("val_blue_x10").at().setI(curr.blue[1], acqTimer, true);
                cntr.pHD[i_p].at().vlAt("val_blue_x200").at().setI(curr.blue[2], acqTimer, true);
                cntr.pHD[i_p].at().vlAt("val_blue_ref").at().setI(curr.blue[3], acqTimer, true);
                cntr.pHD[i_p].at().vlAt("val_none_x1").at().setI(curr.none[0], acqTimer, true);
                cntr.pHD[i_p].at().vlAt("val_none_x10").at().setI(curr.none[1], acqTimer, true);
                cntr.pHD[i_p].at().vlAt("val_none_x200").at().setI(curr.none[2], acqTimer, true);
                cntr.pHD[i_p].at().vlAt("val_none_ref").at().setI(curr.none[3], acqTimer, true);
            }
        } catch(TError &err) { mess_err(err.cat.c_str(), "%s", err.mess.c_str()); }
	cntr.callSt = false;
	//cntr.enRes.unlock();
	cntr.tmGath = TSYS::curTime() - t_cnt;

	//!!! Wait for next iteration
	TSYS::taskSleep(cntr.period(), cntr.period()?"":cntr.cron());
    }

    cntr.prcSt = false;

    return NULL;
}

//!!! Processing virtual function for OpenSCADA control interface comands
void TMdContr::cntrCmdProc( XMLNode *opt )
{
    //Get page info
    if(opt->name() == "info") {
	TController::cntrCmdProc(opt);
	ctrMkNode("fld",opt,-1,"/cntr/cfg/SCHEDULE",cfg("SCHEDULE").fld().descr(),startStat()?R_R_R_:RWRWR_,"root",SDAQ_ID,3,
	    "dest","sel_ed","sel_list",TMess::labSecCRONsel(),"help",TMess::labSecCRON());
	ctrMkNode("fld",opt,-1,"/cntr/cfg/PRIOR",cfg("PRIOR").fld().descr(),startStat()?R_R_R_:RWRWR_,"root",SDAQ_ID,1,"help",TMess::labTaskPrior());

	//Fill in SERIAL ID field
    ctrMkNode("fld",opt,-1,"/cntr/cfg/SERIALID",EVAL_STR,startStat()?R_R_R_:RWRWR_,"root",SDAQ_ID,
                  2, "dest", "select", "select", "/cntr/cfg/lsDEVS");
	return;
    }


    //Process command to page
    string a_path = opt->attr("path");
    if(a_path == "/cntr/cfg/lsDEVS" && ctrChkNode(opt)) {

        const char m_textid[] = "OptoController";
        FT_STATUS ftStatus;
        FT_DEVICE_LIST_INFO_NODE *devInfo;
        DWORD numDevs;// create the device information list
        ftStatus = FT_CreateDeviceInfoList(&numDevs);
        if (ftStatus == FT_OK) {
//            printf("Number of devices is %d\n",numDevs);
            mess_note(nodePath().c_str(), "Found %d laserbox devices", numDevs);
        }

        if (numDevs > 0) {// allocate storage for list based on numDevs
            devInfo = (FT_DEVICE_LIST_INFO_NODE*)malloc(sizeof(FT_DEVICE_LIST_INFO_NODE)*numDevs); // get the device information list
            ftStatus = FT_GetDeviceInfoList(devInfo, &numDevs);
            if (ftStatus == FT_OK) {
                for (int i = 0; i < numDevs; i++) {
                    if (!strcmp(devInfo[i].Description, m_textid)) {
                        opt->childAdd("el")->setText(devInfo[i].SerialNumber);
                    }
                }
            }
        }
    }
    else TController::cntrCmdProc(opt);
//    TController::cntrCmdProc(opt);
}

bool TMdContr::cfgChange( TCfg &co, const TVariant &pc )
{
    TController::cfgChange(co, pc);

    if(co.fld().name() == "SCHEDULE")
	mPer = TSYS::strSepParse(cron(),1,' ').empty() ? vmax(0,(int64_t)(1e9*s2r(cron()))) : 0;

    // Stop acquisition in case SERIALID changed
    if(startStat() && (co.name() == "SERIALID")) stop();

    return true;
}


//*************************************************
//* TMdPrm                                        *
//*************************************************
//!!! Constructor for DAQ-subsystem parameter object.
TMdPrm::TMdPrm( string name, TTypeParam *tp_prm ) :
    TParamContr(name,tp_prm), pEl("w_attr")
{
    pEl.fldAdd(new TFld("val_red_x1",_("Value RED x1"),TFld::Integer,TFld::NoWrite,"",
                        ll2s(EVAL_INT).c_str()));
    pEl.fldAdd(new TFld("val_red_x10",_("Value RED x10"),TFld::Integer,TFld::NoWrite,"",
                        ll2s(EVAL_INT).c_str()));
    pEl.fldAdd(new TFld("val_red_x200",_("Value RED x200"),TFld::Integer,TFld::NoWrite,"",
                        ll2s(EVAL_INT).c_str()));
    pEl.fldAdd(new TFld("val_red_ref",_("Value RED ref"),TFld::Integer,TFld::NoWrite,"",
                        ll2s(EVAL_INT).c_str()));
    pEl.fldAdd(new TFld("val_blue_x1",_("Value BLUE x1"),TFld::Integer,TFld::NoWrite,"",
                        ll2s(EVAL_INT).c_str()));
    pEl.fldAdd(new TFld("val_blue_x10",_("Value BLUE x10"),TFld::Integer,TFld::NoWrite,"",
                        ll2s(EVAL_INT).c_str()));
    pEl.fldAdd(new TFld("val_blue_x200",_("Value BLUE x200"),TFld::Integer,TFld::NoWrite,"",
                        ll2s(EVAL_INT).c_str()));
    pEl.fldAdd(new TFld("val_blue_ref",_("Value BLUE ref"),TFld::Integer,TFld::NoWrite,"",
                        ll2s(EVAL_INT).c_str()));
    pEl.fldAdd(new TFld("val_none_x1",_("Value BACKGROUND x1"),TFld::Integer,TFld::NoWrite,"",
                        ll2s(EVAL_INT).c_str()));
    pEl.fldAdd(new TFld("val_none_x10",_("Value BACKGROUND x10"),TFld::Integer,TFld::NoWrite,"",
                        ll2s(EVAL_INT).c_str()));
    pEl.fldAdd(new TFld("val_none_x200",_("Value BACKGROUND x200"),TFld::Integer,TFld::NoWrite,"",
                        ll2s(EVAL_INT).c_str()));
    pEl.fldAdd(new TFld("val_none_ref",_("Value BACKGROUND ref"),TFld::Integer,TFld::NoWrite,"",
                        ll2s(EVAL_INT).c_str()));
}

//!!! Destructor for DAQ-subsystem parameter object.
TMdPrm::~TMdPrm( )
{
    //!!! Disable and all child nodes early deletion for prevent access to data the object from included nodes on destruction.
    disable();
    nodeDelAll();
}

//!!! Post-enable processing virtual function
void TMdPrm::postEnable( int flag )
{
    TParamContr::postEnable(flag);
    if(!vlElemPresent(&pEl)) vlElemAtt(&pEl);
}

//!!! Direct link to parameter's owner controller
//TMdContr &TMdPrm::owner( )	{ return (TMdContr&)TParamContr::owner(); }
TMdContr &TMdPrm::owner( ) const	{ return (TMdContr&)TParamContr::owner(); }

//!!! Processing virtual functions for enable parameter
void TMdPrm::enable( )
{
    if(enableStat())	return;

    TParamContr::enable();

    owner().prmEn(this, true);
}

//!!! Processing virtual functions for disable parameter
void TMdPrm::disable( )
{
    if(!enableStat())	return;

    owner().prmEn(this, false);

    TParamContr::disable();

    //Set EVAL to the parameter attributes
    vector<string> ls;
    elem().fldList(ls);
    for(int i_el = 0; i_el < ls.size(); i_el++)
	vlAt(ls[i_el]).at().setS(EVAL_STR, 0, true);
}

//!!! Processing virtual functions for load parameter from DB
void TMdPrm::load_( )
{
//    TParamContr::load_();
}

//!!! Processing virtual functions for save parameter to DB
void TMdPrm::save_( )
{
    TParamContr::save_();
}

//!!! Processing virtual function for OpenSCADA control interface comands
void TMdPrm::cntrCmdProc( XMLNode *opt )
{
    //Service commands process
    string a_path = opt->attr("path");
    if(a_path.substr(0,6) == "/serv/")	{ TParamContr::cntrCmdProc(opt); return; }

    //Get page info
    if(opt->name() == "info") {
	    TParamContr::cntrCmdProc(opt);
	    ctrMkNode("fld",opt,-1,"/prm/cfg/OID_LS",cfg("OID_LS").fld().descr(),enableStat()?R_R_R_:RWRWR_,"root",SDAQ_ID);
    	return;
    }

    //Process command to page
    if(a_path == "/prm/cfg/OID_LS" && ctrChkNode(opt,"set",RWRWR_,"root",SDAQ_ID,SEC_WR)) {
	    if(enableStat()) throw TError(nodePath().c_str(),"Parameter is enabled.");
    //	parseOIDList(opt->text());
    } else
        TParamContr::cntrCmdProc(opt);
}

//!!! Processing virtual function for setup archive's parameters which associated with the parameter on time archive creation
void TMdPrm::vlArchMake( TVal &val )
{
    TParamContr::vlArchMake(val);

    if(val.arch().freeStat()) return;
    val.arch().at().setSrcMode(TVArchive::PassiveAttr);
    val.arch().at().setPeriod((int64_t)(owner().period()*1000000));
    val.arch().at().setHardGrid(true);
    val.arch().at().setHighResTm(true);
}
