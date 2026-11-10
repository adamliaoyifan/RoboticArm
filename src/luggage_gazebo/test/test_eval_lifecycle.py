import threading
from types import SimpleNamespace
from luggage_gazebo.eval_lifecycle import close_driver_workers


def test_workers_join_before_nodes_destroyed():
    events=[]
    def worker(name, stop):
        stop.wait()
        events.append(name+'_stopped')
    main_stop=threading.Event();tf_stop=threading.Event()
    spinner=threading.Thread(target=worker,args=('main',main_stop));spinner.start()
    tf_thread=threading.Thread(target=worker,args=('tf',tf_stop));tf_thread.start()
    def destroy(name):
        assert not spinner.is_alive() and not tf_thread.is_alive()
        events.append(name+'_destroyed')
    executor=SimpleNamespace(shutdown=main_stop.set,remove_node=lambda node:events.append('removed'))
    driver=SimpleNamespace(_tf_listener=SimpleNamespace(executor=SimpleNamespace(shutdown=tf_stop.set),
        dedicated_listener_thread=tf_thread),_tf_node=SimpleNamespace(destroy_node=lambda:destroy('tf')),
        destroy_node=lambda:destroy('main'))
    close_driver_workers(driver,executor,spinner)
    assert events==['main_stopped','tf_stopped','removed','main_destroyed','tf_destroyed']
