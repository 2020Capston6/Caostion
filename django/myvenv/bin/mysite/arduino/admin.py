from django.contrib import admin
from .models import Post

#admin.site.register(Post)
# Register your models here.

class PostAdmin(admin.ModelAdmin):
	list_display=['ID','LAT','LONG','ALT','Dat','TEXT']
admin.site.register(Post,PostAdmin)
"""
list_display : Admin 목록에 보여질 필드 목록
list_display_links : 목록 내에서 링크로 지정할 필드 목록 (이를 지정하지 않으면, 첫번째 필드에만 링크가 적용)
list_editable : 목록 상에서 수정할 필드 목록
list_per_page : 페이지 별로 보여질 최대 갯수 (디폴트 : 100)
list_filter : 필터 옵션을 제공할 필드 목록
actions : 목록에서 수행할 action 목록
"""
