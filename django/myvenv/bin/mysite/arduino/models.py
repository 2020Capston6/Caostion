from django.conf import settings
from django.db import models
from django.utils import timezone

# Create your models here.
class Post(models.Model):
    NUM=models.AutoField(primary_key=True)
    ID = models.IntegerField(null=True)
    TEXT = models.TextField(null=True)
    LAT = models.DecimalField(null=True,decimal_places=20,max_digits=40)
    LONG = models.DecimalField(null=True,decimal_places=20,max_digits=40)
    ALT=models.DecimalField(null=True,decimal_places=20,max_digits=40)
    EMER=models.BinaryField(default=0)
    Dat=models.DateTimeField(default=timezone.now)
    
    def __str__(self) :
        return str(self.ID)
