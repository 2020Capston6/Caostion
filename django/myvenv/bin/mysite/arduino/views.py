from django.shortcuts import render
from django.http import QueryDict
from django.http import HttpResponse
import json
from django.views.decorators.csrf import csrf_exempt
from .models import Post
# Create your views here.
@csrf_exempt
def index(request):
    print(request.GET.get('LAT')+" "+request.GET.get('LONG'));

#Data=Post.objects.all()
#   context={'Data':Data}
#  try:
    obj=Post(ID = request.GET.get('ID'), TEXT = request.GET.get('TEXT'),LAT = request.GET.get('LAT'), LONG = request.GET.get('LONG'),ALT=request.GET.get("ALT"))
    print(obj)
    obj.save()
#   except:
#       Datas=None
#       print("??")
#    return render(request,'arduino/index.html',context)
    return HttpResponse("Hello, world. You're at the polls index.")

