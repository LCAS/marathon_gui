import roslib
import rospy
import web
import os
import strands_webserver.client_utils
import strands_webserver.page_utils
from strands_executive_msgs.msg import Task, TaskEvent
from marathon_gui.srv import ShowPageService, ShowPageServiceResponse,\
    CreatePageService, CreatePageServiceResponse, GetPageService,\
    GetPageServiceResponse, IsDefaultService, IsDefaultServiceResponse
from std_srvs.srv import Empty, EmptyResponse

class PageUtils(object):
    """docstring for PageUtils"""
    def __init__(self, package_name='marathon_gui', page='index.html', default_page='index.html', display_no=0):
        self.template_dir = roslib.packages.get_pkg_dir(package_name) + '/templates'
        self.www_prefix = roslib.packages.get_pkg_dir(package_name) + '/www/'
        self.page = page
        self.default_page = default_page
        self.display_no = display_no
        self.history = []
        self.history.append(default_page)
        strands_webserver.client_utils.set_http_root(roslib.packages.get_pkg_dir(package_name) + '/www')
        s = rospy.Service('~show_default', Empty, self.show_default)
        s = rospy.Service('~show_previous', Empty, self.show_previous_page)
        s = rospy.Service('~show_page', ShowPageService, self.show_page)
        s = rospy.Service('~create_page', CreatePageService, self.create_page)
        s = rospy.Service('~get_page', GetPageService, self.get_page)
        s = rospy.Service('~is_default_page', IsDefaultService, self.is_default)


    def generate_interface_page(self, file_name):
        """ Create a page by rendering the marathon_gui/templates/index.html using web.py and place it at marathon_gui/www/ """
        self.history.append(file_name)
        in_full_path = self.www_prefix + file_name
        out_full_path = self.www_prefix + self.page
        try:
            os.makedirs(os.path.dirname(out_full_path))
        except OSError, e:
            pass

        with open(in_full_path) as f:
            lines = f.readlines()
            with open(out_full_path, "w+") as f1:
                f1.writelines(lines)

        self.show()

    def set_default_page(self, file_name):
        """docstring for set_default_page"""
        self.default_page = file_name

    def set_display_no(self, no):
        """docstring for set_default_page"""
        self.display_no = no

    def get_page(self, req):
        return GetPageServiceResponse(self.history[-1])

    def is_default(self, req):
        return IsDefaultServiceResponse(self.history[-1] == self.default_page)

    def show_default(self, req):
        self.generate_interface_page(self.default_page)
        self.show()
        return EmptyResponse()

    def show_page(self, req):
        self.generate_interface_page(req.page)
        self.show()
        return ShowPageServiceResponse()

    def show_previous_page(self, req):
        self.generate_interface_page(self.history[-2] if len(self.history) > 1 else self.history[-1])
        self.show()
        return EmptyResponse()

    def show(self):
        strands_webserver.client_utils.display_relative_page(self.display_no, self.page)

    def create_page(self, req):
        out_full_path = self.www_prefix + self.page
        with open(self.www_prefix+'/'+req.template, 'r') as t:
            page = t.read()
        text = req.text.replace("\n", "<br>")
        text = "<p style=\"font-size:200%;font-family:verdana;color:white;\">" + req.text + "</p>"
        page = page.replace('@BODY_TEXT@', text)
        with open(out_full_path, 'w+') as p:
            p.write(page)
        self.show()
        return CreatePageServiceResponse()
