:- module('spec', [trace_expression/2, match/2]).
:- use_module(monitor('deep_subdict')).
match(_event, acc(I)) :- deep_subdict(_{'input_msg_id':I,'topic':"/battery_monitor/input_accepted"}, _event).
match(_event, acc) :- match(_event, acc(_)).
match(_event, out(I)) :- deep_subdict(_{'input_msg_id':I,'topic':"/battery_monitor/battery_status"}, _event).
match(_event, inOK) :- deep_subdict(_{'percentage':Val,'topic':"/battery_monitor/input_accepted"}, _event), ','((Val>40), (Val=<100)).
match(_event, inMC) :- deep_subdict(_{'percentage':Val,'topic':"/battery_monitor/input_accepted"}, _event), ','((Val>30), (Val=<40)).
match(_event, inSC) :- deep_subdict(_{'percentage':Val,'topic':"/battery_monitor/input_accepted"}, _event), ','((Val>=0), (Val=<30)).
match(_event, inINVALID) :- deep_subdict(_{'percentage':Val,'topic':"/battery_monitor/input_accepted"}, _event), ;((Val<0), (Val>100)).
match(_event, outOK) :- deep_subdict(_{'status':1,'topic':"/battery_monitor/battery_status"}, _event).
match(_event, outMC) :- deep_subdict(_{'status':2,'topic':"/battery_monitor/battery_status"}, _event).
match(_event, outSC) :- deep_subdict(_{'status':3,'topic':"/battery_monitor/battery_status"}, _event).
match(_event, outINVALID) :- deep_subdict(_{'status':4,'topic':"/battery_monitor/battery_status"}, _event).
match(_, any).
trace_expression('Main', Main) :- Main=(CorrectIndices/\clos(star(CompatibleOut))), AccID=gen(['i'], (((acc(var(i)):eps)*app(AccID, [(var('i')+1)]))\/eps)), OutID=gen(['i'], (((out(var(i)):eps)*app(OutID, [(var('i')+1)]))\/eps)), CorrectIndices=((acc>>((acc(1):eps)*app(AccID, [2])));((out(1):eps)*app(OutID, [2]))), CompatibleOut=(((((inOK:eps)*(outOK:eps))\/((inMC:eps)*(outMC:eps)))\/((inSC:eps)*(outSC:eps)))\/((inINVALID:eps)*(outINVALID:eps))).
